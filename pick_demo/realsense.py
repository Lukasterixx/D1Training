"""The wrist RealSense as the pick needs it: colour and depth, aligned, with this camera's own calibration.

`d1_ui/camera_feed.py` already opens a RealSense for the console's camera window, but colour only: enough
to draw YOLO's boxes, not enough to place a cup. The pick's `Frame` needs depth in metres on the same
pixel grid as the colour image, which is `rs.align(rs.stream.color)`, and it needs intrinsics that
describe *this* camera rather than the datasheet.

Why the calibration matters. `camera.CAMERAS["d435"]` derives its intrinsics from the datasheet's
1920x1080 field of view, scaled to 480 rows. Measured on D435I 238222076237 that is wrong in two ways:
the 640x480 colour mode is not a scaled 1920x1080 crop (55.6 deg horizontal, not 69.4), and the
principal point is not the image centre (323.0, 254.3 rather than 320, 240). The 14 px error in cy alone
displaces a cup by about 9 mm at 40 cm. `capture_calibration` reads the real numbers off the device and
`camera_model` turns them into the `CameraModel` the rest of the pick already speaks.

    python -m pick_demo.realsense calibrate      # read this camera and save it
    python -m pick_demo.realsense probe          # what is connected, and what can it stream
    python -m pick_demo.realsense preview        # a few aligned frames, with depth coverage

pyrealsense2 is imported inside the functions that need it, so importing this module costs nothing and
works on a machine with no camera and no SDK -- the same rule `camera.py` and `perception.py` follow.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import numpy as np

from .camera import CameraModel

ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_DIR = ROOT / "pick_demo" / "assets" / "calibration"

# Intel's stereo matcher searches this many disparities, as in `camera.py`: the nearest depth it can
# report is where that search runs out.
DISPARITY_SEARCH = 126
# Datasheet ranging limit for a D435; the calibration cannot report it, so it stays a stated assumption.
DEFAULT_MAX_DEPTH_M = 3.0
# Intel's stereo RMS subpixel error, as `camera.CameraModel.depth_noise_std_m` uses it.
DEFAULT_SUBPIXEL_RMS = 0.08


def _rs():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:     # the solver, the geometry and the tests must run without the SDK
        raise RuntimeError("pyrealsense2 is not installed here, so there is no RealSense") from exc
    return rs


def list_devices() -> list[dict]:
    """Every connected camera, with the fields that identify a run's hardware."""
    rs = _rs()
    out = []
    for device in rs.context().query_devices():
        def info(field, default=None):
            try:
                return device.get_info(field)
            except Exception:
                return default
        out.append({
            "name": info(rs.camera_info.name),
            "serial": info(rs.camera_info.serial_number),
            "firmware": info(rs.camera_info.firmware_version),
            "usb": info(rs.camera_info.usb_type_descriptor),
        })
    return out


def _intrinsics_dict(intr) -> dict:
    return {
        "width": int(intr.width), "height": int(intr.height),
        "fx": float(intr.fx), "fy": float(intr.fy), "cx": float(intr.ppx), "cy": float(intr.ppy),
        "model": str(intr.model), "coeffs": [float(c) for c in intr.coeffs],
        "hfov_deg": round(2 * math.degrees(math.atan(intr.width / (2 * intr.fx))), 3),
        "vfov_deg": round(2 * math.degrees(math.atan(intr.height / (2 * intr.fy))), 3),
    }


def capture_calibration(width: int = 640, height: int = 480, fps: int = 30) -> dict:
    """Start the camera briefly and read what it knows about itself.

    Everything in the returned dict comes off the device: the colour and depth intrinsics for the
    resolution the pick actually streams, the depth-to-colour extrinsics (whose x component is where
    `camera_body.COLOUR_FROM_LEFT_IMAGER_M` comes from), the stereo baseline from the two infrared
    streams, and the depth scale. Intrinsics are per-resolution, so a calibration captured at one size
    does not describe another.
    """
    rs = _rs()
    if not rs.context().query_devices():
        raise RuntimeError("no RealSense connected")
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.infrared, 1, width, height, rs.format.y8, fps)
    config.enable_stream(rs.stream.infrared, 2, width, height, rs.format.y8, fps)
    profile = pipeline.start(config)
    try:
        device = profile.get_device()

        def info(field, default=None):
            try:
                return device.get_info(field)
            except Exception:
                return default

        colour_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        left = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        right = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()

        extrinsics = depth_profile.get_extrinsics_to(colour_profile)
        # librealsense stores the rotation column-major; transpose to the usual row-major matrix.
        rotation = np.array(extrinsics.rotation, dtype=float).reshape(3, 3).T
        translation = np.array(extrinsics.translation, dtype=float)
        baseline = float(abs(np.array(left.get_extrinsics_to(right).translation, dtype=float)[0]))

        return {
            "captured_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "device": {"name": info(rs.camera_info.name), "serial": info(rs.camera_info.serial_number),
                       "firmware": info(rs.camera_info.firmware_version),
                       "usb": info(rs.camera_info.usb_type_descriptor)},
            "stream": {"width": width, "height": height, "fps": fps},
            "colour_intrinsics": _intrinsics_dict(colour_profile.get_intrinsics()),
            "depth_intrinsics": _intrinsics_dict(depth_profile.get_intrinsics()),
            "depth_to_colour": {"translation_m": [float(v) for v in translation],
                                "rotation": [[float(v) for v in row] for row in rotation]},
            "stereo_baseline_m": baseline,
            "depth_scale_m": float(device.first_depth_sensor().get_depth_scale()),
            "source": "read from the connected device through librealsense",
        }
    finally:
        pipeline.stop()


def default_calibration_path(serial: str, width: int = 640, height: int = 480) -> Path:
    return CALIBRATION_DIR / f"d435i_{serial}_{width}x{height}.json"


def save_calibration(calibration: dict, path: str | Path | None = None) -> Path:
    if path is None:
        stream = calibration["stream"]
        path = default_calibration_path(calibration["device"]["serial"], stream["width"], stream["height"])
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(calibration, indent=2) + "\n")
    return path


def load_calibration(path: str | Path) -> dict:
    return json.loads(Path(path).read_text())


def camera_model(calibration: dict, max_depth_m: float = DEFAULT_MAX_DEPTH_M,
                 subpixel_rms: float = DEFAULT_SUBPIXEL_RMS) -> CameraModel:
    """A `CameraModel` describing the camera the calibration came from.

    The intrinsics and the baseline are the device's own. `min_depth_m` is still the disparity-search
    formula rather than a measurement -- it is the nearest depth the matcher *can* report, which no
    calibration field states -- and `max_depth_m` remains the datasheet's. Both are named in `source`
    so a run's record says which parts of its camera model were measured.
    """
    colour, depth = calibration["colour_intrinsics"], calibration["depth_intrinsics"]
    baseline = float(calibration["stereo_baseline_m"])
    min_depth = depth["fx"] * baseline / DISPARITY_SEARCH
    device = calibration["device"]
    return CameraModel(
        name=f"{device.get('name', 'realsense')} #{device.get('serial', '?')}",
        width=int(colour["width"]), height=int(colour["height"]),
        fx=float(colour["fx"]), fy=float(colour["fy"]), cx=float(colour["cx"]), cy=float(colour["cy"]),
        min_depth_m=round(float(min_depth), 4), max_depth_m=float(max_depth_m),
        baseline_m=baseline, depth_fx=float(depth["fx"]), subpixel_rms=float(subpixel_rms),
        source=(f"measured: librealsense on {device.get('name')} #{device.get('serial')} "
                f"at {colour['width']}x{colour['height']}, captured {calibration.get('captured_utc')}; "
                f"min_depth is the {DISPARITY_SEARCH}-disparity limit and max_depth the datasheet's"),
    )


def load_camera_model(path: str | Path, **kwargs) -> CameraModel:
    return camera_model(load_calibration(path), **kwargs)


@dataclass
class RealSenseFrame:
    rgb: np.ndarray        # (H, W, 3) uint8, RGB order
    depth: np.ndarray      # (H, W) float32 metres along the optical axis, 0 where invalid
    stamp_s: float         # the device's own frame timestamp, seconds


class RealSenseCamera:
    """Colour and depth from one camera, depth aligned onto the colour pixels.

    Aligned, because `perception.masked_points` indexes depth with the *colour* image's mask pixels.
    Depth arrives as uint16 device units and leaves as float32 metres with invalid pixels at 0, which
    is the convention `camera.realsense_depth` established for the simulated frames and therefore what
    `perception` already expects.

    One process at a time can hold a RealSense. The console's camera window opens one for its MJPEG
    stream, so a pick running beside it must share that pipeline rather than construct a second
    `RealSenseCamera` -- see `d1_ui/camera_feed.py`.
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30, serial: str | None = None):
        self.width, self.height, self.fps, self.serial = width, height, fps, serial
        self.name = "RealSense"
        self.calibration: dict | None = None
        self._pipeline = None
        self._align = None
        self._depth_scale = 0.001

    def open(self) -> "RealSenseCamera":
        rs = _rs()
        devices = rs.context().query_devices()
        if not devices:
            raise RuntimeError("no RealSense connected")
        pipeline, config = rs.pipeline(), rs.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        profile = pipeline.start(config)
        device = profile.get_device()
        self._depth_scale = float(device.first_depth_sensor().get_depth_scale())
        self.name = (f"{device.get_info(rs.camera_info.name)} "
                     f"#{device.get_info(rs.camera_info.serial_number)}")
        colour_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self.calibration = {
            "device": {"name": device.get_info(rs.camera_info.name),
                       "serial": device.get_info(rs.camera_info.serial_number)},
            "stream": {"width": self.width, "height": self.height, "fps": self.fps},
            "colour_intrinsics": _intrinsics_dict(colour_profile.get_intrinsics()),
            "depth_intrinsics": _intrinsics_dict(depth_profile.get_intrinsics()),
            "depth_scale_m": self._depth_scale,
        }
        self._align = rs.align(rs.stream.color)
        self._pipeline = pipeline
        return self

    def model(self, **kwargs) -> CameraModel:
        """This camera's `CameraModel`, from the profile read when it was opened.

        The stereo baseline is not among the streams opened here, so it falls back to the datasheet's
        50 mm; `capture_calibration` measures it when the infrared streams are on.
        """
        if self.calibration is None:
            raise RuntimeError("open() first")
        calibration = dict(self.calibration)
        calibration.setdefault("stereo_baseline_m", 0.050)
        calibration.setdefault("captured_utc", None)
        return camera_model(calibration, **kwargs)

    def read(self, timeout_s: float = 1.0) -> RealSenseFrame | None:
        if self._pipeline is None:
            raise RuntimeError("open() first")
        frames = self._pipeline.wait_for_frames(int(timeout_s * 1000))
        frames = self._align.process(frames)
        colour, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not colour or not depth:
            return None
        rgb = np.asanyarray(colour.get_data()).copy()
        metres = np.asanyarray(depth.get_data()).astype(np.float32) * self._depth_scale
        return RealSenseFrame(rgb, metres, float(colour.get_timestamp()) / 1000.0)

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            finally:
                self._pipeline, self._align = None, None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()


def _main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=("calibrate", "probe", "preview"))
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--out", default=None, help="Where to write the calibration (default: pick_demo/assets/calibration/).")
    ap.add_argument("--frames", type=int, default=30, help="preview: how many aligned frames to read.")
    args = ap.parse_args(argv)

    if args.command == "probe":
        devices = list_devices()
        if not devices:
            print("no RealSense connected")
            return 1
        for device in devices:
            print(f"{device['name']}  serial {device['serial']}  firmware {device['firmware']}  USB {device['usb']}")
            if device["usb"] and device["usb"].startswith("2"):
                print("  note: enumerated at USB 2 -- fewer stream combinations and lower rates than USB 3")
        return 0

    if args.command == "calibrate":
        calibration = capture_calibration(args.width, args.height, args.fps)
        path = save_calibration(calibration, args.out)
        model = camera_model(calibration)
        colour = calibration["colour_intrinsics"]
        print(f"{calibration['device']['name']} #{calibration['device']['serial']} "
              f"(firmware {calibration['device']['firmware']}, USB {calibration['device']['usb']})")
        print(f"  colour {colour['width']}x{colour['height']}: fx {colour['fx']:.2f} fy {colour['fy']:.2f} "
              f"cx {colour['cx']:.2f} cy {colour['cy']:.2f}  ({colour['hfov_deg']:.1f} x {colour['vfov_deg']:.1f} deg)")
        print(f"  depth: fx {calibration['depth_intrinsics']['fx']:.2f}, baseline "
              f"{1000 * calibration['stereo_baseline_m']:.2f} mm, scale {calibration['depth_scale_m']:.6f} m/unit")
        offset = calibration["depth_to_colour"]["translation_m"]
        print(f"  depth -> colour: {[round(1000 * v, 3) for v in offset]} mm")
        print(f"  nearest depth the matcher can report: {model.min_depth_m:.3f} m")
        print(f"saved {path}")
        return 0

    with RealSenseCamera(args.width, args.height, args.fps) as camera:
        print(f"streaming from {camera.name}")
        import time

        started, read = time.monotonic(), 0
        coverage = []
        for _ in range(args.frames):
            frame = camera.read(timeout_s=5.0)
            if frame is None:
                continue
            read += 1
            coverage.append(float((frame.depth > 0).mean()))
        elapsed = time.monotonic() - started
        if not read:
            print("no frames")
            return 1
        valid = np.array(coverage)
        print(f"{read} aligned frames in {elapsed:.2f} s ({read / elapsed:.1f} fps)")
        print(f"depth coverage {100 * valid.mean():.1f}% of pixels (min {100 * valid.min():.1f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())

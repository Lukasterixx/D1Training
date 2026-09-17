"""The console's camera window: wrist frames with YOLO's boxes drawn on, served as MJPEG.

One background thread reads a frame source, runs the detector on the frame, draws the boxes onto that same
frame and JPEG-encodes it, so every image the page shows carries the detections for exactly that image.
The page gets it from `/camera.mjpg`; `status()` goes out with the state so the page can list what was seen.

Sources:
* `SimFrameSource` -- the simulator's rendered wrist camera, through the sim feed (`sim_feed.py`).
* `RealSenseSource` -- the wrist RealSense through pyrealsense2: colour, and depth aligned onto the colour
  pixels when `want_depth` is on.

**One process at a time can hold a RealSense**, so this pipeline is the only thing that opens it. The
hardware pick does not construct a second camera; it reads frames from here through `latest_frame()`,
which is why the source carries depth at all. A source may return `rgb` alone or `(rgb, depth)`; the
pipeline accepts both, draws boxes on the colour image either way, and keeps the depth for whoever asks.

The detector is `pick_demo.perception.YoloDetector`, the same stock COCO weights the scripted pick uses, so
a box here is what the pick would see. Everything degrades rather than fails: no pyrealsense2 means no
frames, no ultralytics means frames without boxes, and the reason is in `status()` for the page to show.
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = ROOT / "generated/yolo/yolo11s-seg.pt"

# BGR, as OpenCV draws: the labels the user asked for in green, anything else in amber.
_TARGET_BGR = (60, 200, 60)
_OTHER_BGR = (0, 200, 230)


class SimFrameSource:
    def __init__(self, client):
        self.client = client
        self.name = "simulator wrist camera (rendered)"

    def open(self) -> None:
        health = self.client.health()
        if health is None:
            raise RuntimeError("no simulator feed")
        if not health.get("camera"):
            raise RuntimeError(f"this simulation ({health.get('source')}) has no wrist camera; "
                               "./run_pick_demo.sh has one")

    def read(self, timeout_s: float = 1.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            frame = self.client.fetch_frame()
            if frame is not None:
                return frame[0]
            time.sleep(0.02)
        return None

    def close(self) -> None:
        pass


class RealSenseSource:
    """The first RealSense found: colour, plus depth aligned to it when `want_depth` is on.

    Depth costs USB bandwidth and is only needed when something is going to place an object in space, so
    the console's camera window can run without it. The pick needs it, and needs it *aligned*, because
    `perception.masked_points` indexes depth with the colour image's own mask pixels.

    `calibration` is filled in when the camera opens, from the profile the device reports, so the pick can
    build a `CameraModel` describing the camera actually streaming rather than a datasheet preset (F-053).
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30, want_depth: bool = False):
        self.width, self.height, self.fps = width, height, fps
        self.want_depth = want_depth
        self.name = "RealSense"
        self.calibration: dict | None = None
        self._pipeline = None
        self._align = None
        self._depth_scale = 0.001

    def open(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("pyrealsense2 is not installed here, so there is no RealSense stream") from exc
        if len(rs.context().query_devices()) == 0:
            raise RuntimeError("no RealSense connected")
        pipeline, config = rs.pipeline(), rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        if self.want_depth:
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        profile = pipeline.start(config)
        device = profile.get_device()
        self.name = f"{device.get_info(rs.camera_info.name)} #{device.get_info(rs.camera_info.serial_number)}"
        colour_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        calibration = {
            "device": {"name": device.get_info(rs.camera_info.name),
                       "serial": device.get_info(rs.camera_info.serial_number)},
            "stream": {"width": self.width, "height": self.height, "fps": self.fps},
            "colour_intrinsics": _intrinsics_dict(colour_profile.get_intrinsics()),
        }
        if self.want_depth:
            self._depth_scale = float(device.first_depth_sensor().get_depth_scale())
            depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            calibration["depth_intrinsics"] = _intrinsics_dict(depth_profile.get_intrinsics())
            calibration["depth_scale_m"] = self._depth_scale
            self._align = rs.align(rs.stream.color)
        self.calibration = calibration
        self._pipeline = pipeline

    def read(self, timeout_s: float = 1.0):
        frames = self._pipeline.wait_for_frames(int(timeout_s * 1000))
        if self._align is not None:
            frames = self._align.process(frames)
        colour = frames.get_color_frame()
        if not colour:
            return None
        rgb = np.asanyarray(colour.get_data()).copy()
        if self._align is None:
            return rgb
        depth = frames.get_depth_frame()
        if not depth:
            return rgb, None
        return rgb, np.asanyarray(depth.get_data()).astype(np.float32) * self._depth_scale

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            finally:
                self._pipeline, self._align = None, None


def _intrinsics_dict(intr) -> dict:
    """The fields `pick_demo.realsense.camera_model` needs, in its own shape."""
    import math

    return {"width": int(intr.width), "height": int(intr.height),
            "fx": float(intr.fx), "fy": float(intr.fy), "cx": float(intr.ppx), "cy": float(intr.ppy),
            "model": str(intr.model), "coeffs": [float(c) for c in intr.coeffs],
            "hfov_deg": round(2 * math.degrees(math.atan(intr.width / (2 * intr.fx))), 3),
            "vfov_deg": round(2 * math.degrees(math.atan(intr.height / (2 * intr.fy))), 3)}


def draw_detections(bgr, detections, targets):
    """Boxes, mask outlines and labels onto a BGR image, in place."""
    import cv2

    for detection in detections:
        colour = _TARGET_BGR if detection.label in targets else _OTHER_BGR
        if detection.mask is not None:
            contours, _ = cv2.findContours(detection.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(bgr, contours, -1, colour, 1)
        x1, y1, x2, y2 = (int(round(c)) for c in detection.box)
        cv2.rectangle(bgr, (x1, y1), (x2, y2), colour, 2)
        text = f"{detection.label} {detection.confidence:.2f}"
        (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(y1 - h - 6, 0)
        cv2.rectangle(bgr, (x1, top), (x1 + w + 6, top + h + 6), colour, -1)
        cv2.putText(bgr, text, (x1 + 3, top + h + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)
    return bgr


def make_yolo(weights, device: str = "auto", confidence: float = 0.25):
    """The pick's detector on the best device here."""
    import sys

    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    try:
        import ultralytics  # noqa: F401
    except ImportError as exc:
        raise RuntimeError("ultralytics is not installed here") from exc
    from pick_demo.perception import YoloDetector, ensure_weights

    if Path(weights) == DEFAULT_WEIGHTS and not DEFAULT_WEIGHTS.is_file():
        try:
            ensure_weights(DEFAULT_WEIGHTS)
        except Exception as exc:     # e.g. the Jetson, with no route to GitHub: run_ui.sh deploys them instead
            raise RuntimeError(f"no YOLO weights at {weights}, and downloading them failed ({exc})") from exc
    if not Path(weights).is_file():
        raise RuntimeError(f"no YOLO weights at {weights}")

    if device == "auto":
        import torch

        device = "cuda:0" if torch.cuda.is_available() else "cpu"
    detector = YoloDetector(str(weights), device=device, confidence=confidence)
    detector.weights_name = Path(weights).name
    return detector


class CameraPipeline:
    """source -> detector -> boxes -> JPEG, on its own thread. Reopens the source when it drops."""

    def __init__(self, source, detector_factory=None, targets=("cup",), max_fps: float = 15.0, log=print,
                 jpeg_quality: int = 80):
        self.source, self.detector_factory = source, detector_factory
        self.targets = tuple(targets)
        self.period = 1.0 / max_fps
        self.log = log
        self.jpeg_quality = jpeg_quality
        self.detector = None
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._frame = None          # (rgb, depth, monotonic stamp) of the most recent read
        self._seq = 0
        self._status = {"available": False, "source": source.name, "message": "starting", "targets": list(self.targets),
                        "detector": None, "detector_ready": False, "detections": [], "fps": None, "detect_ms": None,
                        "frame_age_s": None}
        self._last_frame = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="d1-ui-camera")

    def start(self) -> "CameraPipeline":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def status(self) -> dict:
        with self._cond:
            status = dict(self._status)
        status["frame_age_s"] = round(time.monotonic() - self._last_frame, 2) if self._last_frame else None
        return status

    def _set(self, **fields) -> None:
        with self._cond:
            self._status.update(fields)

    def wait_jpeg(self, after_seq: int, timeout_s: float = 5.0):
        """(seq, jpeg) for a frame newer than `after_seq`, or None on timeout."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq > after_seq or self._stop.is_set(), timeout=timeout_s):
                return None
            return (self._seq, self._jpeg) if self._jpeg is not None else None

    def latest_jpeg(self):
        with self._cond:
            return self._jpeg

    def latest_frame(self):
        """(seq, rgb, depth, stamp) of the newest frame read, or None. `depth` is None without one.

        This is how the hardware pick gets its images: the pipeline owns the camera, so the pick reads
        the frames it has already captured instead of opening a second one, which the device would
        refuse. Arrays are handed out as read: nothing downstream may write to them.
        """
        with self._cond:
            if self._frame is None:
                return None
            rgb, depth, stamp = self._frame
            return self._seq, rgb, depth, stamp

    def wait_frame(self, after_seq: int, timeout_s: float = 2.0):
        """The first frame newer than `after_seq`, so a caller never reads the same image twice."""
        with self._cond:
            if not self._cond.wait_for(lambda: (self._seq > after_seq and self._frame is not None)
                                       or self._stop.is_set(), timeout=timeout_s):
                return None
            if self._frame is None:
                return None
            rgb, depth, stamp = self._frame
            return self._seq, rgb, depth, stamp

    def has_depth(self) -> bool:
        with self._cond:
            return self._frame is not None and self._frame[1] is not None

    def _load_detector(self) -> None:
        if self.detector_factory is None:
            self._set(detector="off (--detect none)")
            return
        self._set(detector="loading YOLO")
        try:
            self.detector = self.detector_factory()
            name = getattr(self.detector, "weights_name", "detector")
            self._set(detector=f"{name} on {getattr(self.detector, 'device', '?')}", detector_ready=True)
            self.log(f"camera: detector ready ({name})")
        except Exception as exc:    # frames still flow, without boxes
            self.detector = None
            self._set(detector=f"unavailable: {exc}")
            self.log(f"camera: no detector ({exc}); showing frames without boxes")

    def _run(self) -> None:
        try:
            import cv2
        except ImportError:
            self._set(available=False, message="OpenCV is not installed here, so there is no camera window")
            self.log("camera: OpenCV is not installed; no camera window")
            return
        threading.Thread(target=self._load_detector, daemon=True, name="d1-ui-yolo-load").start()
        opened, last_error, streaming = False, None, False
        stamps: list[float] = []
        while not self._stop.is_set():
            if not opened:
                try:
                    self.source.open()
                    opened, last_error = True, None
                    self._set(available=True, source=self.source.name, message="waiting for frames")
                except Exception as exc:
                    if str(exc) != last_error:
                        self.log(f"camera: {exc}; retrying")
                        last_error = str(exc)
                    self._set(available=False, message=str(exc))
                    self._stop.wait(2.0)
                    continue
            started = time.monotonic()
            try:
                read = self.source.read(timeout_s=1.0)
                # A source may hand back colour alone or (colour, depth); both are normal.
                rgb, depth = read if isinstance(read, tuple) else (read, None)
            except Exception as exc:
                self.log(f"camera: stream dropped ({exc})")
                self.source.close()
                opened, streaming = False, False
                continue
            if rgb is None:
                if self._last_frame and time.monotonic() - self._last_frame > 2.0:
                    if streaming:
                        self.log("camera: no frames for 2 s")
                        streaming = False
                    self._set(message="no new frames")
                    # The source may have gone away (a simulator closing): check it is still there.
                    self.source.close()
                    opened = False
                continue
            if not streaming:
                self.log(f"camera: streaming from {self.source.name}")
                streaming = True

            detections, detect_ms = [], None
            detector = self.detector
            if detector is not None:
                t0 = time.monotonic()
                try:
                    labels = tuple(detector.names.values()) if "all" in self.targets else self.targets
                    detections = detector.detect(rgb, labels=labels)
                except Exception as exc:
                    self._set(detector=f"failed: {exc}", detector_ready=False)
                    self.detector = None
                detect_ms = round(1000 * (time.monotonic() - t0), 1)

            bgr = draw_detections(np.ascontiguousarray(rgb[..., ::-1]), detections, self.targets)
            ok, jpeg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            now = time.monotonic()
            stamps = [s for s in stamps if now - s < 2.0] + [now]
            if ok:
                with self._cond:
                    self._jpeg = jpeg.tobytes()
                    self._frame = (rgb, depth, now)
                    self._seq += 1
                    self._status.update(
                        available=True, message=None, detect_ms=detect_ms, depth=depth is not None,
                        fps=round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 1) if len(stamps) > 2 else None,
                        width=int(rgb.shape[1]), height=int(rgb.shape[0]),
                        detections=[{"label": d.label, "confidence": round(d.confidence, 3),
                                     "box": [round(c, 1) for c in d.box]} for d in detections])
                    self._cond.notify_all()
                self._last_frame = now
            self._stop.wait(max(0.0, self.period - (time.monotonic() - started)))
        self.source.close()

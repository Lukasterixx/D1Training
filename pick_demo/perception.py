"""From a wrist frame to a cup position in the Go2 base frame: YOLO for *which* pixels, depth for *where*.

YOLO is a stock COCO segmentation model (`cup` is class 41) with no training. It answers only which
pixels belong to the cup. Geometry does the rest: the masked pixels are deprojected with depth, moved
into the base frame through the arm's forward kinematics and the wrist mount, and a circle is fitted to
the ones at rim height. The rim is used rather than the whole cloud because it is the one part of a cup
seen from above whose horizontal centre is the axis: the near wall sits a radius in front of it, the
inside a wall's thickness behind, and the handle to one side.

When the camera is too close for depth (a D435 below ~18 cm), `ray_to_height` recovers the position
from the mask alone: the ray through the mask's centre meets the rim plane, whose height is already
known from the first, longer look.

Only `YoloDetector` imports ultralytics, and only when constructed, so the geometry runs anywhere.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import os

import numpy as np

from .camera import CameraModel, WristMount, camera_pose, deproject, pixel_ray

COCO_CUP = "cup"
# Seen steeply from above a cup is a ring, and COCO models often call that a bowl. Accepted only for
# re-detection near a cup already found, never for the first detection.
TOP_VIEW_LABELS = ("cup", "bowl", "vase", "wine glass")


@dataclass
class Frame:
    """One wrist camera frame and the arm state it was captured at."""

    rgb: np.ndarray          # (H, W, 3) uint8, RGB order
    depth: np.ndarray        # (H, W) float32 metres along the optical axis, 0 where invalid
    q: np.ndarray            # (6,) joint angles (URDF radians) from feedback at capture
    up_b: np.ndarray         # (3,) world up in the base frame, from the IMU's gravity estimate
    stamp_s: float


@dataclass
class Detection:
    label: str
    confidence: float
    box: tuple[float, float, float, float]   # x1, y1, x2, y2 pixels
    mask: np.ndarray | None                   # (H, W) bool

    def mask_centroid(self) -> tuple[float, float]:
        if self.mask is not None and self.mask.any():
            v, u = np.nonzero(self.mask)
            return float(u.mean()), float(v.mean())
        x1, y1, x2, y2 = self.box
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0


@dataclass
class CupEstimate:
    """A standing cup in the base frame. `top_centre_b` is on the axis at rim height."""

    top_centre_b: np.ndarray
    radius_m: float
    top_height_m: float       # along `up`, in the base frame
    bottom_height_m: float
    points: int
    method: str               # "rim_circle", "silhouette" or "ray_to_height"
    residual_rms_m: float | None = None
    rim_coverage_deg: float | None = None

    @property
    def height_m(self) -> float:
        return self.top_height_m - self.bottom_height_m

    def as_dict(self) -> dict:
        return {"top_centre_b": [round(float(v), 5) for v in self.top_centre_b],
                "radius_m": round(self.radius_m, 5), "top_height_m": round(self.top_height_m, 5),
                "bottom_height_m": round(self.bottom_height_m, 5), "height_m": round(self.height_m, 5),
                "points": self.points, "method": self.method,
                "residual_rms_m": None if self.residual_rms_m is None else round(self.residual_rms_m, 5),
                "rim_coverage_deg": None if self.rim_coverage_deg is None else round(self.rim_coverage_deg, 1)}


@dataclass
class CupObservation:
    detection: Detection
    estimate: CupEstimate
    camera_pose_b: np.ndarray
    valid_depth_fraction: float
    notes: list[str] = field(default_factory=list)


def horizontal_basis(up):
    up = np.asarray(up, dtype=float)
    up = up / np.linalg.norm(up)
    seed = np.array([1.0, 0.0, 0.0]) if abs(up[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = seed - up * (seed @ up)
    e1 /= np.linalg.norm(e1)
    return up, e1, np.cross(up, e1)


def fit_circle(xy):
    """Algebraic (Kasa) least-squares circle: centre (2,), radius, rms residual."""
    xy = np.asarray(xy, dtype=float)
    a = np.column_stack([xy[:, 0], xy[:, 1], np.ones(len(xy))])
    b = -(xy[:, 0] ** 2 + xy[:, 1] ** 2)
    (d, e, f), *_ = np.linalg.lstsq(a, b, rcond=None)
    centre = np.array([-d / 2.0, -e / 2.0])
    radius = float(np.sqrt(max(centre @ centre - f, 0.0)))
    residual = np.linalg.norm(xy - centre, axis=1) - radius
    return centre, radius, float(np.sqrt(np.mean(residual ** 2))), residual


def estimate_cup(points_b, up_b, camera_pos_b, rim_band_m: float = 0.006,
                 radius_range_m=(0.01, 0.10)) -> CupEstimate | None:
    """Fit a standing cup to masked points (N, 3) in the base frame. None if there is too little to fit."""
    points_b = np.asarray(points_b, dtype=float)
    if len(points_b) < 30:
        return None
    up, e1, e2 = horizontal_basis(up_b)
    heights = points_b @ up
    # The rim is a thin ring, ~2% of a cup's masked pixels from 40 cm, so a 98th percentile lands under it.
    top = float(np.percentile(heights, 99.5))
    bottom = float(np.percentile(heights, 1.0))   # reported only; the grasp is placed from the rim
    band = points_b[heights > top - rim_band_m]
    flat = np.column_stack([band @ e1, band @ e2])

    if len(flat) >= 20:
        # Trimmed refits: the handle's top and the inside of the rim fall in the band too.
        keep = np.ones(len(flat), dtype=bool)
        for _ in range(4):
            centre, radius, _, residual = fit_circle(flat[keep])
            spread = 1.4826 * np.median(np.abs(residual - np.median(residual)))
            refit = np.abs(np.linalg.norm(flat - centre, axis=1) - radius) < max(2.5 * spread, 0.003)
            if refit.sum() < 20 or np.array_equal(refit, keep):
                break
            keep = refit
        centre, radius, rms, _ = fit_circle(flat[keep])
        if keep.sum() >= 20 and radius_range_m[0] <= radius <= radius_range_m[1] and rms < 0.005:
            angles = np.sort(np.arctan2(*(flat[keep] - centre).T[::-1]))
            gaps = np.diff(np.concatenate([angles, angles[:1] + 2 * np.pi]))
            coverage = float(np.degrees(2 * np.pi - gaps.max()))
            top_centre = centre[0] * e1 + centre[1] * e2 + top * up
            return CupEstimate(top_centre, radius, top, bottom, len(points_b), "rim_circle", rms, coverage)

    # Silhouette fallback: the lateral middle of the cloud, pushed back by a radius from its near face.
    flat_all = np.column_stack([points_b @ e1, points_b @ e2])
    cam = np.array([camera_pos_b @ e1, camera_pos_b @ e2])
    view = flat_all.mean(axis=0) - cam
    view /= max(np.linalg.norm(view), 1e-9)
    lateral = np.array([-view[1], view[0]])
    along, across = flat_all @ view, flat_all @ lateral
    lo, hi = np.percentile(across, [5.0, 95.0])
    radius = float((hi - lo) / 2.0)
    if not radius_range_m[0] <= radius <= radius_range_m[1]:
        return None
    centre = ((lo + hi) / 2.0) * lateral + (np.percentile(along, 5.0) + radius) * view
    top_centre = centre[0] * e1 + centre[1] * e2 + top * up
    return CupEstimate(top_centre, radius, top, bottom, len(points_b), "silhouette")


def ray_to_height(model: CameraModel, cam_pose_b, u: float, v: float, up_b, height: float):
    """Where the ray through pixel (u, v) meets the plane at `height` along up; None if it points away."""
    up = np.asarray(up_b, dtype=float) / np.linalg.norm(up_b)
    direction = cam_pose_b[:3, :3] @ pixel_ray(model, u, v)
    origin = cam_pose_b[:3, 3]
    denom = float(direction @ up)
    if abs(denom) < 1e-6:
        return None
    t = (height - float(origin @ up)) / denom
    return None if t <= 0.0 else origin + t * direction


def masked_points(model: CameraModel, detection: Detection, depth, stride: int = 2):
    """Deprojected points (N, 3) in the optical frame for valid-depth pixels of the mask, and the valid fraction."""
    mask = detection.mask
    if mask is None:
        x1, y1, x2, y2 = (int(round(c)) for c in detection.box)
        mask = np.zeros(depth.shape, dtype=bool)
        mask[max(y1, 0):y2, max(x1, 0):x2] = True
    sub = np.zeros_like(mask)
    sub[::stride, ::stride] = mask[::stride, ::stride]
    v, u = np.nonzero(sub)
    if len(u) == 0:
        return np.zeros((0, 3)), 0.0
    z = depth[v, u]
    ok = z > 0.0
    return deproject(model, u[ok], v[ok], z[ok]), float(ok.mean())


class YoloDetector:
    """Stock ultralytics segmentation weights, run on RGB frames. Imports ultralytics on construction.

    `YOLO_AUTOINSTALL` is forced off: ultralytics otherwise pip-installs whatever it thinks is missing
    into the running environment, which here would pull numpy 2 into Isaac Sim's.
    """

    def __init__(self, weights: str, device: str = "cuda:0", confidence: float = 0.25, image_size: int = 640):
        os.environ["YOLO_AUTOINSTALL"] = "False"
        from ultralytics import YOLO

        self.model = YOLO(weights)
        self.device, self.confidence, self.image_size = device, confidence, image_size
        self.names = self.model.names

    def detect(self, rgb, labels=(COCO_CUP,)) -> list[Detection]:
        import cv2

        wanted = [i for i, name in self.names.items() if name in labels]
        # ultralytics treats numpy input as OpenCV BGR.
        result = self.model.predict(rgb[..., ::-1].copy(), classes=wanted, conf=self.confidence,
                                    imgsz=self.image_size, device=self.device, verbose=False, retina_masks=True)[0]
        detections = []
        height, width = rgb.shape[:2]
        for i in range(len(result.boxes)):
            mask = None
            if result.masks is not None:
                mask = np.zeros((height, width), dtype=np.uint8)
                polygon = result.masks.xy[i]
                if len(polygon) >= 3:
                    cv2.fillPoly(mask, [np.round(polygon).astype(np.int32)], 1)
                mask = mask.astype(bool)
            detections.append(Detection(self.names[int(result.boxes.cls[i])], float(result.boxes.conf[i]),
                                        tuple(float(c) for c in result.boxes.xyxy[i].tolist()), mask))
        return sorted(detections, key=lambda d: -d.confidence)


class CupPerception:
    """Detector plus geometry: `observe` finds a cup anywhere, `reobserve` looks for it again near a prior."""

    def __init__(self, detector, model: CameraModel, joints, mount: WristMount, min_confidence: float = 0.4):
        self.detector, self.model, self.joints, self.mount = detector, model, joints, mount
        self.min_confidence = min_confidence
        self.last_detections: list[Detection] = []

    def observe(self, frame: Frame) -> CupObservation | None:
        detections = self.detector.detect(frame.rgb, labels=(COCO_CUP,))
        self.last_detections = detections
        pose = camera_pose(self.joints, frame.q, self.mount)
        for detection in detections:
            if detection.confidence < self.min_confidence:
                continue
            points, valid = masked_points(self.model, detection, frame.depth)
            points_b = points @ pose[:3, :3].T + pose[:3, 3]
            estimate = estimate_cup(points_b, frame.up_b, pose[:3, 3])
            if estimate is not None:
                return CupObservation(detection, estimate, pose, valid)
        return None

    def reobserve(self, frame: Frame, prior: CupEstimate, gate_m: float = 0.04) -> CupObservation | None:
        """Re-detect near `prior`. Depth when there is enough of it; otherwise the mask's centre, cast onto
        the prior's rim plane."""
        detections = self.detector.detect(frame.rgb, labels=TOP_VIEW_LABELS)
        self.last_detections = detections
        pose = camera_pose(self.joints, frame.q, self.mount)
        up = np.asarray(frame.up_b, dtype=float) / np.linalg.norm(frame.up_b)
        for detection in detections:
            if detection.confidence < 0.25:
                continue
            points, valid = masked_points(self.model, detection, frame.depth)
            estimate, notes = None, []
            if valid >= 0.5 and len(points) >= 60:
                points_b = points @ pose[:3, :3].T + pose[:3, 3]
                estimate = estimate_cup(points_b, up, pose[:3, 3])
                if estimate is not None and abs(estimate.top_height_m - prior.top_height_m) > 0.02:
                    notes.append(f"depth rim height {estimate.top_height_m:.3f} disagrees with prior; using ray")
                    estimate = None
            if estimate is None:
                u, v = detection.mask_centroid()
                hit = ray_to_height(self.model, pose, u, v, up, prior.top_height_m)
                if hit is None:
                    continue
                estimate = CupEstimate(hit, prior.radius_m, prior.top_height_m, prior.bottom_height_m,
                                       int(detection.mask.sum()) if detection.mask is not None else 0,
                                       "ray_to_height")
            shift = estimate.top_centre_b - prior.top_centre_b
            shift -= up * (shift @ up)
            if np.linalg.norm(shift) <= gate_m:
                observation = CupObservation(detection, estimate, pose, valid, notes)
                return observation
        return None

"""RealSense camera models and the wrist mount, in plain numpy.

Frames. The camera's optical frame is the ROS one: x right, y down, z forward, the same convention
librealsense deprojects into and Isaac Lab's `CameraCfg.OffsetCfg(convention="ros")` takes. Poses are
4x4 transforms into the Go2 base frame, which is `d1_ik`'s frame.

**Nothing here is calibrated.** The presets are derived from Intel's datasheet fields of view and
stereo baselines; on the real camera, replace the intrinsics with the ones librealsense reports
(`profile.as_video_stream_profile().get_intrinsics()`) and the mount with a measurement of the bracket.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from position_only.workspace import forward

# Intel's stereo matcher searches 126 disparities at the default depth unit; the nearest depth it can
# report is where that search runs out: min_z = f_depth * baseline / 126.
_DISPARITY_SEARCH = 126


@dataclass(frozen=True)
class CameraModel:
    """Colour intrinsics (depth is aligned to colour, as `rs.align` does) and the depth limits."""

    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    min_depth_m: float
    max_depth_m: float
    baseline_m: float
    depth_fx: float
    subpixel_rms: float
    source: str

    @property
    def intrinsic_matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def depth_noise_std_m(self, z):
        """Intel's stereo RMS error model, sigma_z = z^2 * subpixel / (f * baseline). Best case: a
        textured, well-lit, fronto-parallel surface. Real surfaces are noisier."""
        z = np.asarray(z, dtype=float)
        return z * z * self.subpixel_rms / (self.depth_fx * self.baseline_m)


def _colour_fx(width_native: int, hfov_deg: float, scale: float) -> float:
    """Focal length of a native colour stream scaled by `scale` (librealsense resizes, then crops)."""
    return (width_native / 2.0) / math.tan(math.radians(hfov_deg) / 2.0) * scale


def _min_z(depth_width: int, depth_hfov_deg: float, baseline_m: float) -> tuple[float, float]:
    f = (depth_width / 2.0) / math.tan(math.radians(depth_hfov_deg) / 2.0)
    return f, f * baseline_m / _DISPARITY_SEARCH


def _preset(name, native_w, native_h, colour_hfov, depth_hfov, depth_width, baseline, max_depth, source):
    # 640x480 colour: scale the native stream to 480 rows, then crop the width to 640.
    scale = 480.0 / native_h
    fx = _colour_fx(native_w, colour_hfov, scale)
    depth_fx, min_z = _min_z(depth_width, depth_hfov, baseline)
    return CameraModel(name, 640, 480, fx, fx, 320.0, 240.0, round(min_z, 3), max_depth, baseline, depth_fx,
                       0.08, source)


CAMERAS = {
    # D435/D435i: colour 69.4 x 42.5 deg at 1920x1080, depth 87 x 58 deg, 50 mm baseline. Depth at the
    # 848x480 Intel recommends gives a min-Z of about 18 cm.
    "d435": _preset("d435", 1920, 1080, 69.4, 87.0, 848, 0.050, 3.0,
                    "derived from the Intel D400 datasheet FOV and baseline; not a calibration"),
    # D455: colour 90 x 65 deg at 1280x800, depth 87 x 58 deg, 95 mm baseline: min-Z about 34 cm.
    "d455": _preset("d455", 1280, 800, 90.0, 87.0, 848, 0.095, 4.0,
                    "derived from the Intel D400 datasheet FOV and baseline; not a calibration"),
    # D405: one imager for colour and depth, 87 x 58 deg, 18 mm baseline, rated 7-50 cm. The close-range
    # wrist camera. Its min-Z is the datasheet's, not the disparity formula's.
    "d405": CameraModel("d405", 640, 480, _colour_fx(1280, 87.0, 480.0 / 720.0), _colour_fx(1280, 87.0, 480.0 / 720.0),
                        320.0, 240.0, 0.07, 0.50, 0.018, _min_z(848, 87.0, 0.018)[0], 0.08,
                        "Intel D405 datasheet FOV, baseline and 7-50 cm rated range; not a calibration"),
}


def rotation_about(axis, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle_rad) * k + (1.0 - math.cos(angle_rad)) * (k @ k)


def transform(rot, pos) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = pos
    return out


def invert(pose) -> np.ndarray:
    rot, pos = pose[:3, :3], pose[:3, 3]
    return transform(rot.T, -rot.T @ pos)


@dataclass(frozen=True)
class WristMount:
    """The camera's optical frame in the Link6 frame.

    Link6's z axis is the approach axis (the fingers point along it), its y axis is the jaw axis, and
    with the gripper level its -x axis is up. The default puts the camera on that upper face, 5.5 cm off
    the approach axis and 3.5 cm along it -- just behind the finger roots, clear of the Link6 shell
    (x from -3.8 to +1.9 cm in the CAD mesh) -- pitched 20 deg towards the axis so the jaws sit in the
    lower half of the image. **Assumed**, not measured: it stands in for a bracket that does not exist yet.
    """

    pos_link6: tuple[float, float, float] = (-0.055, 0.0, 0.035)
    pitch_deg: float = 20.0

    @property
    def rotation(self) -> np.ndarray:
        """Columns are the optical x (right), y (down) and z (forward) axes in Link6."""
        pitch = math.radians(self.pitch_deg)
        z = np.array([math.sin(pitch), 0.0, math.cos(pitch)])   # forward, tilted towards +x (the axis)
        y = np.array([math.cos(pitch), 0.0, -math.sin(pitch)])  # image down: +x, down when level
        return np.column_stack([np.cross(y, z), y, z])

    @property
    def pose(self) -> np.ndarray:
        return transform(self.rotation, self.pos_link6)

    def quat_wxyz(self) -> tuple[float, float, float, float]:
        return matrix_to_quat_wxyz(self.rotation)


@dataclass(frozen=True)
class MeasuredMount:
    """A mount known as a whole transform in the Link6 frame, as a hand-eye calibration reports it."""

    matrix: tuple            # 16 floats, row-major 4x4
    source: str

    @classmethod
    def from_pose(cls, pose, source: str) -> "MeasuredMount":
        return cls(tuple(float(v) for v in np.asarray(pose, dtype=float).reshape(16)), source)

    @property
    def pose(self) -> np.ndarray:
        return np.array(self.matrix).reshape(4, 4)

    @property
    def rotation(self) -> np.ndarray:
        return self.pose[:3, :3]

    @property
    def pos_link6(self) -> tuple[float, float, float]:
        return tuple(float(v) for v in self.pose[:3, 3])


def matrix_to_quat_wxyz(rot) -> tuple[float, float, float, float]:
    rot = np.asarray(rot, dtype=float)
    trace = np.trace(rot)
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w, x, y, z = 0.25 / s, (rot[2, 1] - rot[1, 2]) * s, (rot[0, 2] - rot[2, 0]) * s, (rot[1, 0] - rot[0, 1]) * s
    else:
        i = int(np.argmax(np.diag(rot)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(rot[i, i] - rot[j, j] - rot[k, k] + 1.0, 0.0)) * 2.0
        q = [0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (rot[j, i] + rot[i, j]) / s
        q[k] = (rot[k, i] + rot[i, k]) / s
        w = (rot[k, j] - rot[j, k]) / s
        x, y, z = q
    q = np.array([w, x, y, z])
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in (q if q[0] >= 0.0 else -q))


def link6_pose(joints, q) -> np.ndarray:
    frames, _, _ = forward(joints, np.asarray(q, dtype=float).reshape(1, 6))
    rot, pos = frames["Link6"]
    return transform(rot[0], pos[0])


def camera_pose(joints, q, mount: WristMount) -> np.ndarray:
    """Optical frame in the Go2 base frame, from joint angles. On the real arm `q` is the feedback."""
    return link6_pose(joints, q) @ mount.pose


def deproject(model: CameraModel, u, v, depth) -> np.ndarray:
    """Pixel indices and depth along the optical axis -> points (N, 3) in the optical frame.

    Pixel (u, v) is the centre of that pixel, hence the half-pixel offset against a principal point
    measured from the image corner.
    """
    u, v, depth = (np.asarray(a, dtype=float) for a in (u, v, depth))
    x = (u + 0.5 - model.cx) / model.fx * depth
    y = (v + 0.5 - model.cy) / model.fy * depth
    return np.stack([x, y, depth], axis=-1)


def project(model: CameraModel, points_cam) -> np.ndarray:
    """Points (N, 3) in the optical frame -> continuous pixel coordinates (N, 2), before the half-pixel."""
    points_cam = np.asarray(points_cam, dtype=float).reshape(-1, 3)
    z = points_cam[:, 2]
    return np.stack([model.fx * points_cam[:, 0] / z + model.cx - 0.5,
                     model.fy * points_cam[:, 1] / z + model.cy - 0.5], axis=-1)


def pixel_ray(model: CameraModel, u: float, v: float) -> np.ndarray:
    """Unit ray through a pixel centre, in the optical frame."""
    ray = np.array([(u + 0.5 - model.cx) / model.fx, (v + 0.5 - model.cy) / model.fy, 1.0])
    return ray / np.linalg.norm(ray)


def realsense_depth(model: CameraModel, depth_true, rng: np.random.Generator | None = None) -> np.ndarray:
    """What the camera would report for a perfect rendered depth: zero outside its range (as librealsense
    marks invalid pixels), Gaussian noise from `depth_noise_std_m` inside it.

    Not modelled: edge flying pixels, holes on dark or shiny surfaces, projector pattern, temporal noise
    correlation. A perception result in simulation is therefore an upper bound on the real one.
    """
    depth = np.asarray(depth_true, dtype=np.float32).copy()
    valid = np.isfinite(depth) & (depth >= model.min_depth_m) & (depth <= model.max_depth_m)
    if rng is not None:
        noise = rng.standard_normal(depth.shape).astype(np.float32)
        depth = np.where(valid, depth + noise * model.depth_noise_std_m(np.where(valid, depth, 0.0)), 0.0)
    return np.where(valid, depth, 0.0).astype(np.float32)

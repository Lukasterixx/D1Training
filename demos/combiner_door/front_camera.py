"""The Go2's own front camera: where it is, the lens model the look measures with, and how the simulator draws it.

The combiner door, key and insertion tasks find the box's AprilTag with this camera (since 2026-10-22). The RealSense
moved to the D1's wrist on 2026-10-21, and a look through it places the tag through the bracket's CAD mount and the
arm's joints: the door pull went in 3-5 cm low and the insertion's camera saw the lock ~25-30 degrees off where the
mount put it (`results/week_06/notes.md`, 2026-10-21). This camera is in the trunk, so the look needs no arm move and
nothing between the tag and the base but its own fixed mount. The wrist RealSense is left the insertion's keyhole view.

**The mount** is Unitree's URDF (`description/go2_d1.urdf`, P2Dingo's `config/go2.urdf` alike): `Head_upper` at
(0.285, 0, 0.01) in `base_link`, `front_camera` (0.045, 0, 0.03) further, rpy (-90, 0, -90): the optical frame (x right,
y down, z forward) at (0.330, 0, 0.040), looking straight ahead. The lens behind the face's cover is not measured.

**The lens** is a wide one with plain barrel distortion (P2Dingo's live view, 2026-10-22), 1280x720 as the head streams
it, ~100 x 53 degrees. It is modelled as OpenCV's fisheye (equidistant, `cv2.fisheye`): r = f * theta_d, theta_d = theta * (1 + k1 theta^2
+ k2 theta^4 + k3 theta^6 + k4 theta^8). **NOT A FULL CALIBRATION** (Lukas, 2026-10-22: "use a nominal model for now;
calibrate properly later"). `ONE_STAND` is fitted to one stand on the dog: the camera taped 0.61 m from the box's front,
square to it, 20 frames. f and k1 were searched so that the box's tag lies on a plane 0.61 m off (0.609 fitted) and the
box's 0.63 m width, its two side edges carried onto that plane, comes out 0.63 m (0.629), the edges straight (1.7 mm).
f is pinned by the distance (15 px = 1 cm of it), k1 by the width (0.03 = 7 mm of it). Two checks it was not fitted to:
the MID-360 put the door's plane 0.603 m from the camera's URDF position, and the camera saw the tag 4.7 degrees off
square where the lidar saw the face tilted 4.72 (so the camera's mount agrees with the lidar's in pitch). The principal
point is assumed at the image centre and k2-k4 at 0: with two quantities measured, two parameters fitted. Before it
(`NOMINAL`, f 830, k1 0, from a rough door-edge check) the same stand read 0.629 m and a 0.605 m door. Replace it with a
calibration (OpenCV's `cv2.fisheye.calibrate` on a chessboard, the same model) before trusting a height from it far
off the image's centre.

**On the dog** the frames come from P2Dingo's `p2dingo-front-camera` container: the head's H.264 multicast decoded and
published as JPEG. `TOPIC` is its local full-resolution topic (q90); the 960 px one is for the zenoh link to the PC.

**In the simulator** the lens is drawn in two steps: Isaac renders a pinhole camera wide enough to hold the fisheye's
whole field (`render_cfg`, `RENDER_FOCAL_SCALE` of its focal length), and `distort_render` remaps that into the fisheye
image the detector then sees, the same 1280x720 the dog streams.

Numpy at import; `distort_render` imports OpenCV, `render_cfg` Isaac, when called.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from . import head_camera

#: The optical frame's origin in `base_link`: `Head_upper_joint` (0.285, 0, 0.01) + `front_camera_joint` (0.045, 0, 0.03).
OPTICAL_POSITION_BASE_M = (0.285 + 0.045, 0.0, 0.01 + 0.03)
#: The optical frame (x right, y down, z forward) in `base_link`, looking along +x: the URDF's rpy (-90, 0, -90), the same
#: rotation as the head RealSense's (`head_camera.OPTICAL_QUAT_BASE_WXYZ`), written w first.
OPTICAL_QUAT_BASE_WXYZ = head_camera.OPTICAL_QUAT_BASE_WXYZ
NEAR_CLIP_M = 0.03
FAR_CLIP_M = 10.0
#: P2Dingo's full-resolution topic (`scripts/go2_front_camera/go2_front_camera_node.py`, `full_topic`).
TOPIC = "/go2_front_camera/image_full/compressed"
#: Isaac's pinhole render's focal length as a fraction of the fisheye's: the render must hold the fisheye's corners
#: (58.8 degrees off axis for `ONE_STAND`, whose field is ~100 x 53 degrees), so it is wider than the output; at 0.75 it
#: is 1714 x 966 and the image's centre is drawn from 0.75 of the output's pixels.
RENDER_FOCAL_SCALE = 0.75
#: The USD camera's focal length (the aperture follows from it and K). Isaac Lab's default makes the horizontal
#: aperture 1.0, which for this wide render puts the focal length at 0.35: Isaac then reports the intrinsics asked for and
#: renders the scene ~1.35x too large (the first sim runs, 2026-10-22: the box 17 cm too near by the front camera, 1 cm by
#: the head RealSense in the same scene, whose focal is 0.95; independent of the render's size and aspect). At 1.8
#: (aperture 5.2) the two agree to 5 mm.
USD_FOCAL_LENGTH = 1.8


@dataclass(frozen=True)
class FisheyeModel:
    """A fisheye camera as OpenCV's `cv2.fisheye` has it: K and k1-k4."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple = (0.0, 0.0, 0.0, 0.0)
    source: str = ""
    calibrated: bool = False

    @property
    def intrinsic_matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def _theta_d(self, theta):
        k1, k2, k3, k4 = self.distortion
        t2 = theta * theta
        return theta * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))

    def project(self, points_cam) -> np.ndarray:
        """Pixels (N, 2) of points (N, 3) in the optical frame (z forward); NaN behind the camera."""
        p = np.atleast_2d(np.asarray(points_cam, dtype=float))
        r = np.hypot(p[:, 0], p[:, 1])
        theta = np.arctan2(r, p[:, 2])
        scale = np.where(r > 1e-12, self._theta_d(theta) / np.maximum(r, 1e-12), 1.0 / np.maximum(p[:, 2], 1e-12))
        uv = np.column_stack((self.fx * p[:, 0] * scale + self.cx, self.fy * p[:, 1] * scale + self.cy))
        uv[p[:, 2] <= 0.0] = np.nan
        return uv

    def rays(self, uv) -> np.ndarray:
        """Unit rays (N, 3) in the optical frame for pixels (N, 2): the inverse of `project` (Newton on theta)."""
        uv = np.atleast_2d(np.asarray(uv, dtype=float))
        x = (uv[:, 0] - self.cx) / self.fx
        y = (uv[:, 1] - self.cy) / self.fy
        theta_d = np.hypot(x, y)
        theta = theta_d.copy()
        k1, k2, k3, k4 = self.distortion
        for _ in range(20):
            t2 = theta * theta
            f = self._theta_d(theta) - theta_d
            df = 1.0 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + t2 * 9 * k4)))
            theta = theta - f / df
        s = np.where(theta_d > 1e-12, np.sin(theta) / np.maximum(theta_d, 1e-12), 1.0)
        return np.column_stack((x * s, y * s, np.cos(theta)))

    def max_angle_rad(self) -> float:
        """The largest angle off axis anywhere in the image (at a corner)."""
        corners = np.array([[0, 0], [self.width, 0], [0, self.height], [self.width, self.height]], dtype=float)
        return float(np.max(np.arccos(np.clip(self.rays(corners)[:, 2], -1.0, 1.0))))


#: The first guess (module docstring): equidistant, f 830 px, centred, k1-k4 = 0. Kept for comparison; not used.
NOMINAL = FisheyeModel(
    width=1280, height=720, fx=830.0, fy=830.0, cx=640.0, cy=360.0, distortion=(0.0, 0.0, 0.0, 0.0),
    source="nominal guess: equidistant fisheye, f 830 px from one rough door-width check against the MID-360 on the "
           "dog (2026-10-22), principal point at the image centre, k1-k4 = 0",
    calibrated=False)

#: Fitted to one stand on the dog (module docstring): f 790 px, k1 -0.09. NOT a full calibration.
ONE_STAND = FisheyeModel(
    width=1280, height=720, fx=790.0, fy=790.0, cx=640.0, cy=360.0, distortion=(-0.09, 0.0, 0.0, 0.0),
    source="NOT a full calibration: f and k1 fitted to one stand on the dog (2026-10-22), the camera taped 0.61 m from "
           "the box's front (tag plane 0.609 m) and the box's 0.63 m width (0.629 m); principal point assumed at the "
           "image centre, k2-k4 = 0; checked against the MID-360's door plane (0.603 m) and tilt (4.72 vs 4.7 deg)",
    calibrated=False)


def camera_model() -> FisheyeModel:
    """The model the look measures with and the simulator draws: `ONE_STAND` until a calibration replaces it."""
    return ONE_STAND


def optical_pose_base() -> np.ndarray:
    """The optical frame in `base_link`, 4x4."""
    pose = np.eye(4)
    pose[:3, :3] = head_camera.quat_to_matrix(OPTICAL_QUAT_BASE_WXYZ)
    pose[:3, 3] = OPTICAL_POSITION_BASE_M
    return pose


def frustum_matrix(model: FisheyeModel) -> np.ndarray:
    """A pinhole K whose image corners look along the fisheye's corner rays: what RViz's camera field (a pyramid,
    `viz.frustum_points`) is drawn with. The fisheye's own K would draw it ~20% too narrow at the corners."""
    corner = model.rays([[0.0, 0.0]])[0]
    fx = model.cx / (-corner[0] / corner[2])
    fy = model.cy / (-corner[1] / corner[2])
    return np.array([[fx, 0.0, model.cx], [0.0, fy, model.cy], [0.0, 0.0, 1.0]])


def in_view(model: FisheyeModel, points_cam, margin_px: float = 0.0) -> np.ndarray:
    """Whether each point (N, 3), optical frame, lands inside the image with `margin_px` to spare."""
    uv = model.project(points_cam)
    ok = np.isfinite(uv).all(axis=1)
    ok &= (uv[:, 0] >= margin_px) & (uv[:, 0] < model.width - margin_px)
    ok &= (uv[:, 1] >= margin_px) & (uv[:, 1] < model.height - margin_px)
    return ok


# --- the simulator --------------------------------------------------------------------------------------------------

def render_size(model: FisheyeModel = None, focal_scale: float = RENDER_FOCAL_SCALE) -> tuple[int, int, float]:
    """The pinhole render that holds the fisheye's whole field: (width, height, focal length), even sizes."""
    model = model or camera_model()
    f = focal_scale * model.fx
    border = np.concatenate([np.column_stack((np.linspace(0, model.width, 64), np.full(64, edge)))
                             for edge in (0.0, float(model.height))]
                            + [np.column_stack((np.full(64, edge), np.linspace(0, model.height, 64)))
                               for edge in (0.0, float(model.width))])
    r = model.rays(border)
    x = np.max(np.abs(r[:, 0] / r[:, 2]))
    y = np.max(np.abs(r[:, 1] / r[:, 2]))
    width = 2 * int(math.ceil(f * x + 2))
    height = 2 * int(math.ceil(f * y + 2))
    return width, height, f


_MAPS: dict = {}


def distort_maps(model: FisheyeModel = None, focal_scale: float = RENDER_FOCAL_SCALE):
    """cv2.remap's maps from the pinhole render (`render_size`) to the fisheye image, cached."""
    model = model or camera_model()
    key = (model, focal_scale)
    if key not in _MAPS:
        width, height, f = render_size(model, focal_scale)
        u, v = np.meshgrid(np.arange(model.width, dtype=float), np.arange(model.height, dtype=float))
        r = model.rays(np.column_stack((u.ravel(), v.ravel())))
        map_x = (f * r[:, 0] / r[:, 2] + width / 2.0).reshape(model.height, model.width).astype(np.float32)
        map_y = (f * r[:, 1] / r[:, 2] + height / 2.0).reshape(model.height, model.width).astype(np.float32)
        _MAPS[key] = (map_x, map_y)
    return _MAPS[key]


def distort_render(rgb, model: FisheyeModel = None, focal_scale: float = RENDER_FOCAL_SCALE) -> np.ndarray:
    """The fisheye image (model.height, model.width, C) from a pinhole render at `render_size`."""
    import cv2

    map_x, map_y = distort_maps(model, focal_scale)
    return cv2.remap(np.ascontiguousarray(rgb), map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def render_cfg(prim_path: str = "/World/envs/env_.*/Robot/base/front_cam", model: FisheyeModel = None,
               focal_scale: float = RENDER_FOCAL_SCALE):
    """Isaac's tiled pinhole camera at the mount, wide enough for `distort_render`, for every environment at once."""
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import TiledCameraCfg

    width, height, f = render_size(model, focal_scale)
    intrinsic = [f, 0.0, width / 2.0, 0.0, f, height / 2.0, 0.0, 0.0, 1.0]
    return TiledCameraCfg(
        prim_path=prim_path,
        update_period=0.0,
        width=width, height=height,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
            intrinsic_matrix=intrinsic, width=width, height=height, clipping_range=(NEAR_CLIP_M, FAR_CLIP_M),
            focal_length=USD_FOCAL_LENGTH),
        offset=TiledCameraCfg.OffsetCfg(pos=OPTICAL_POSITION_BASE_M, rot=OPTICAL_QUAT_BASE_WXYZ, convention="ros"),
    )

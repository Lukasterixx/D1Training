"""The leg task's viewer overlay: the door and its lever handle, which exist only as an analytic plant.

UniFP's overlay (`debug_vis.py`) already draws the foot's goal, the force-displaced target, the controlled point and
the force arrows. This adds, per environment with a handle:

  * **the door's face**, a thin light panel through `door`, normal `n`;
  * **the rose**, a short dark cylinder from the face to the spindle's axis;
  * **the bar**, a cylinder along the lever at its current angle: **grey** at rest, **orange** while turned short of the
    latch, **green** once the latch is drawn back.

For the combiner box's key (`LeverHandle.pad`), instead of the rose and the bar: **the lock adapter**, a dark block on
the face round the shaft, and **the key**, its bar and its platform as two blocks at its current angle, in the same
three colours (grey at the start, orange while turning, green once the door has unlocked).

Nothing is drawn for an environment without a handle. Viewer only: nothing is allocated until a viewer asks for it.
"""
from __future__ import annotations

import torch

from . import handle
from .debug_vis import arrow_pose

DOOR_SIZE_M = (0.008, 0.45, 0.45)
ROSE_RADIUS_M = 0.025
BAR_REST, BAR_TURNING, BAR_OPEN, DOOR, ROSE, KEY_REST, KEY_TURNING, KEY_OPEN, ADAPTER = range(9)
MARKERS_PER_ENV = 4   # door, rose or adapter, bar or key bar, (keys) the key's platform
#: The adapter as drawn: 60 x 60 mm, `leg_cfg.KEY_ADAPTER_FACE_M` proud of the door.
ADAPTER_SIZE_M = 0.060


def marker_cfg(prim_path: str):
    import isaaclab.sim as sim_utils
    from isaaclab.markers import VisualizationMarkersCfg

    def material(colour):
        return sim_utils.PreviewSurfaceCfg(diffuse_color=colour)

    def bar(colour):
        return sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="X", visual_material=material(colour))

    def block(colour):
        return sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=material(colour))

    return VisualizationMarkersCfg(prim_path=prim_path, markers={
        "bar_rest": bar((0.55, 0.55, 0.58)),
        "bar_turning": bar((1.0, 0.5, 0.05)),
        "bar_open": bar((0.1, 0.75, 0.25)),
        "door": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=material((0.85, 0.82, 0.76))),
        "rose": sim_utils.CylinderCfg(radius=ROSE_RADIUS_M, height=1.0, axis="X", visual_material=material((0.2, 0.2, 0.22))),
        "key_rest": block((0.55, 0.55, 0.58)),
        "key_turning": block((1.0, 0.5, 0.05)),
        "key_open": block((0.1, 0.75, 0.25)),
        "adapter": block((0.16, 0.17, 0.19)),
    })


class HandleMarkers:
    def __init__(self, prim_path: str, num_envs: int, device: str) -> None:
        from isaaclab.markers import VisualizationMarkers

        self.markers = VisualizationMarkers(marker_cfg(prim_path))
        self.num_envs, self.device = num_envs, device

    def update(self, hd: handle.LeverHandle, origins: torch.Tensor) -> None:
        n = self.num_envs
        translations = torch.zeros(n, MARKERS_PER_ENV, 3, device=self.device)
        orientations = torch.zeros(n, MARKERS_PER_ENV, 4, device=self.device)
        orientations[..., 0] = 1.0
        scales = torch.ones(n, MARKERS_PER_ENV, 3, device=self.device)
        indices = torch.zeros(n, MARKERS_PER_ENV, dtype=torch.long, device=self.device)
        code = lambda value: torch.full((n,), value, dtype=torch.long, device=self.device)

        quat, _ = arrow_pose(hd.n)
        translations[:, 0] = hd.door + origins - 0.5 * DOOR_SIZE_M[0] * hd.n
        orientations[:, 0] = quat
        scales[:, 0] = torch.tensor(DOOR_SIZE_M, device=self.device)
        indices[:, 0] = code(DOOR)

        translations[:, 1] = 0.5 * (hd.door + hd.pivot) + origins
        orientations[:, 1] = quat
        scales[:, 1, 0] = (hd.pivot - hd.door).norm(dim=-1).clamp(min=1e-3)
        indices[:, 1] = code(ROSE)

        u = handle.lever_dir(hd.u0, hd.theta)
        start, end = hd.pivot, hd.pivot + hd.length.unsqueeze(-1) * u
        quat_bar, length = arrow_pose(end - start)
        translations[:, 2] = 0.5 * (start + end) + origins
        orientations[:, 2] = quat_bar
        scales[:, 2, 0] = length
        scales[:, 2, 1] = hd.radius
        scales[:, 2, 2] = hd.radius
        indices[:, 2] = torch.where(hd.latch_open(), code(BAR_OPEN),
                                    torch.where(hd.theta > 0.02, code(BAR_TURNING), code(BAR_REST)))
        scales[:, 3] = 1.0e-6
        if bool(hd.pad.any()):
            self._keys(hd, origins, translations, orientations, scales, indices, code)
        scales[~hd.active] = 1.0e-6
        self.markers.visualize(translations=translations.view(-1, 3), orientations=orientations.view(-1, 4),
                               scales=scales.view(-1, 3), marker_indices=indices.view(-1))

    def _keys(self, hd, origins, translations, orientations, scales, indices, code) -> None:
        """The key's markers where `hd.pad`: the adapter and the key's two boxes (`leg_cfg.KEY_*_BOX`), each a block
        whose x runs along the key, y up its top side and z off the door."""
        from . import leg_cfg

        key = hd.pad
        u, top, n = handle.lever_dir(hd.u0, hd.theta), handle.lever_top(hd.u0, hd.theta), hd.n
        quat = _frame_quat(u, top, n)
        u0, top0 = hd.u0, torch.zeros_like(hd.u0)
        top0[:, 2] = 1.0
        colour = torch.where(hd.latch_open(), code(KEY_OPEN),
                             torch.where(hd.theta > hd.theta_min + 0.02, code(KEY_TURNING), code(KEY_REST)))
        face = leg_cfg.KEY_ADAPTER_FACE_M
        k = key.unsqueeze(-1)
        translations[:, 1] = torch.where(k, hd.door + origins + 0.5 * face * n, translations[:, 1])
        orientations[:, 1] = torch.where(k, _frame_quat(u0, top0, n), orientations[:, 1])
        scales[:, 1] = torch.where(k, torch.tensor([ADAPTER_SIZE_M, ADAPTER_SIZE_M, face], device=self.device),
                                   scales[:, 1])
        indices[:, 1] = torch.where(key, code(ADAPTER), indices[:, 1])
        for slot, box in ((2, leg_cfg.KEY_BAR_BOX), (3, leg_cfg.KEY_PAD_BOX)):
            (a0, a1), (b0, b1), (c0, c1) = box                        # along the key, off the door, up its top
            centre = (hd.door + 0.5 * (a0 + a1) * u + 0.5 * (b0 + b1) * n + 0.5 * (c0 + c1) * top) + origins
            translations[:, slot] = torch.where(k, centre, translations[:, slot])
            orientations[:, slot] = torch.where(k, quat, orientations[:, slot])
            scales[:, slot] = torch.where(k, torch.tensor([a1 - a0, c1 - c0, b1 - b0], device=self.device),
                                          scales[:, slot])
            indices[:, slot] = torch.where(key, colour, indices[:, slot])

    def set_visibility(self, visible: bool) -> None:
        self.markers.set_visibility(visible)


def _frame_quat(x: torch.Tensor, y: torch.Tensor, z: torch.Tensor) -> torch.Tensor:
    """(w, x, y, z) of the rotations whose columns are the unit vectors `x`, `y`, `z`, (N, 3) each."""
    m = torch.stack((x, y, z), dim=-1)
    w = torch.sqrt((1.0 + m[:, 0, 0] + m[:, 1, 1] + m[:, 2, 2]).clamp(min=1e-12)) / 2
    qx = torch.copysign(torch.sqrt((1.0 + m[:, 0, 0] - m[:, 1, 1] - m[:, 2, 2]).clamp(min=1e-12)) / 2, m[:, 2, 1] - m[:, 1, 2])
    qy = torch.copysign(torch.sqrt((1.0 - m[:, 0, 0] + m[:, 1, 1] - m[:, 2, 2]).clamp(min=1e-12)) / 2, m[:, 0, 2] - m[:, 2, 0])
    qz = torch.copysign(torch.sqrt((1.0 - m[:, 0, 0] - m[:, 1, 1] + m[:, 2, 2]).clamp(min=1e-12)) / 2, m[:, 1, 0] - m[:, 0, 1])
    return torch.stack((w, qx, qy, qz), dim=-1)


# --- cameras for an evaluation's frames ---------------------------------------------------------------------------

#: The fixed side view: from the robot's front right, looking at where the foot meets the handle (env frame).
OVERVIEW_EYE = (0.85, -0.95, 0.62)
OVERVIEW_TARGET = (0.35, -0.10, 0.30)
#: For the key: on the robot's side of the door (0.40-0.47 m ahead), from its right, at the key's height (0.42 m).
KEY_OVERVIEW_EYE = (0.02, -0.85, 0.72)
KEY_OVERVIEW_TARGET = (0.38, -0.12, 0.40)


def wrist_camera_cfg():
    """The cup pick's wrist RealSense (D435 preset, 640 x 480, the saved CAD mount, F-138), RGB only."""
    from demos.cup.pick_demo.camera import CAMERAS, resolve_mount
    from demos.cup.pick_demo.scene import wrist_camera_cfg as pick_wrist_camera_cfg

    mount, _, _ = resolve_mount(None)
    cfg = pick_wrist_camera_cfg(CAMERAS["d435"], mount, link6_path="/World/envs/env_.*/Robot/D1/Link6")
    cfg.data_types = ["rgb"]
    return cfg


def _look_at_quat(eye, target):
    """World-convention camera orientation (+X forward, +Z up) looking from `eye` at `target`, w-first."""
    import numpy as np

    from demos.cup.pick_demo.camera import matrix_to_quat_wxyz

    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    left = np.cross((0.0, 0.0, 1.0), f)
    left /= np.linalg.norm(left)
    up = np.cross(f, left)
    return matrix_to_quat_wxyz(np.column_stack((f, left, up)))


def overview_camera_cfg(eye=OVERVIEW_EYE, target=OVERVIEW_TARGET):
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg

    return CameraCfg(
        prim_path="/World/envs/env_.*/overview_cam", update_period=0.0, width=640, height=480, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, horizontal_aperture=20.955, clipping_range=(0.05, 20.0)),
        offset=CameraCfg.OffsetCfg(pos=eye, rot=_look_at_quat(eye, target), convention="world"))

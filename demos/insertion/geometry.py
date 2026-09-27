"""The key and the keyed slot the insertion demo builds in PhysX, sized as the training's plant (`unifp_train.insertion`).

Slot frame: origin at the centre of the mouth, on the door's face; +x into the door (the insertion axis), +y along the
slot's long side (`u`), +z along its short side (`w`). The door's material is everything at x >= 0 outside the channel.
Numbers are the middle of the training's ranges (`unifp_train.insert_cfg`); none is a measurement of a real lock.
"""
from __future__ import annotations

from dataclasses import dataclass

import torch

from unifp_train import insertion


@dataclass(frozen=True)
class SlotGeometry:
    #: The key blade's half sizes (long, short) and its tip's distance beyond the jaw centre.
    key_half_u: float = insertion.KEY_HALF_U_M
    key_half_w: float = insertion.KEY_HALF_W_M
    key_length: float = insertion.KEY_LENGTH_M
    #: Clearance per side, the mouth's 45° chamfer, the depth to the seat.
    clearance: float = 0.001
    chamfer: float = 0.002
    depth: float = 0.018
    #: How far the lock body extends around the channel, and the back plate's thickness.
    margin: float = 0.06
    back: float = 0.006
    #: The door panel the lock sits in: its half size across, and thickness.
    panel_half: float = 0.18
    panel_thickness: float = 0.012
    friction: float = 0.3

    @property
    def half_u(self) -> float:
        return self.key_half_u + self.clearance

    @property
    def half_w(self) -> float:
        return self.key_half_w + self.clearance


GEOMETRY = SlotGeometry()

#: Where the blade collider sits on `Link6`: from just past the fingertips (their ends are at z = 0.1256, and the
#: blade clears them by 1 mm so the claw's own fingers never touch it) to the tip at the jaw centre + key length.
JAW_CENTRE_Z_M = 0.1051
KEY_ROOT_Z_M = 0.1266


def frame_quat(x: torch.Tensor, y: torch.Tensor, z: torch.Tensor) -> torch.Tensor:
    """(w, x, y, z) quaternions of the rotations whose columns are the unit vectors `x`, `y`, `z`, (N, 3) each: where
    the door is placed so its slot frame's axes are the true slot's axis, long side and short side."""
    m = torch.stack((x, y, z), dim=-1)
    w = torch.sqrt((1.0 + m[:, 0, 0] + m[:, 1, 1] + m[:, 2, 2]).clamp(min=1e-12)) / 2
    qx = torch.sqrt((1.0 + m[:, 0, 0] - m[:, 1, 1] - m[:, 2, 2]).clamp(min=1e-12)) / 2
    qy = torch.sqrt((1.0 - m[:, 0, 0] + m[:, 1, 1] - m[:, 2, 2]).clamp(min=1e-12)) / 2
    qz = torch.sqrt((1.0 - m[:, 0, 0] - m[:, 1, 1] + m[:, 2, 2]).clamp(min=1e-12)) / 2
    qx = torch.copysign(qx, m[:, 2, 1] - m[:, 1, 2])
    qy = torch.copysign(qy, m[:, 0, 2] - m[:, 2, 0])
    qz = torch.copysign(qz, m[:, 1, 0] - m[:, 0, 1])
    return torch.stack((w, qx, qy, qz), dim=-1)

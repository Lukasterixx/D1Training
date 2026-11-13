"""The key held by the arm (`keyhold_env.py`): the grip's geometry, plain torch, no Isaac imports (`tests/test_keyhold.py`)."""
from __future__ import annotations

import torch

from . import leg_cfg


def hand_rotation(approach: torch.Tensor, bar: torch.Tensor, roll: int = leg_cfg.KEYHOLD_GRIP_ROLL) -> torch.Tensor:
    """Link6's rotation (columns x, y, z) holding a bar along `bar`, coming in along `approach`
    (`demos/unifp/decoupled_arm.hand_rotation` at pitch 0), batched: (N, 3), (N, 3) -> (N, 3, 3)."""
    z = approach / approach.norm(dim=-1, keepdim=True)
    y = torch.cross(z, bar, dim=-1)
    y = roll * y / y.norm(dim=-1, keepdim=True)
    x = torch.cross(y, z, dim=-1)
    return torch.stack((x, y, z), dim=-1)


def bar_at(bar0: torch.Tensor, turn: torch.Tensor) -> torch.Tensor:
    """The lever's direction `turn` rad clockwise (as the robot sees it, down) from `bar0` (horizontal)."""
    down = torch.zeros_like(bar0)
    down[:, 2] = -1.0
    return torch.cos(turn).unsqueeze(-1) * bar0 + torch.sin(turn).unsqueeze(-1) * down


def turn_ahead(theta: torch.Tensor, theta_ref: torch.Tensor, theta_start: torch.Tensor,
               free: float = leg_cfg.KEYHOLD_TURN_AHEAD_FREE_RAD) -> torch.Tensor:
    """How far the key runs ahead of the task layer's reference, past `free` (rad, >= 0). While the reference still
    descends onto the key it stands below `theta_start`, and a key at its start is not ahead (F-283)."""
    return torch.relu(theta - torch.maximum(theta_ref, theta_start) - free)


def follow_turn(theta: torch.Tensor, theta_ref: torch.Tensor, theta_start: torch.Tensor, theta_latch: torch.Tensor,
                latch_open: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """`--key_turn_follow_weight`'s progress and latch for the reward (F-283): the key's angle counted only up to the
    reference, and the latch counted open only once the reference has reached it. Turned in one stroke at first
    contact, the key then earns no more than one turned along the arc. No free band here: one would pay the stroke for
    running that far ahead (`turn_ahead`'s band is for the price, where the estimate's error must not be charged)."""
    ref = torch.maximum(theta_ref, theta_start)
    return torch.minimum(theta, ref), latch_open & (ref >= theta_latch)


def grip_point(door: torch.Tensor, normal: torch.Tensor) -> torch.Tensor:
    """The jaw centre holding a seated key whose shaft's axis meets the door's face at `door` (normal out of it)."""
    return door + leg_cfg.KEYHOLD_JAW_OFF_DOOR_M * normal


def quat_matrix(quat: torch.Tensor) -> torch.Tensor:
    """(N, 4) w, x, y, z -> (N, 3, 3)."""
    w, x, y, z = quat.unbind(-1)
    return torch.stack((
        torch.stack((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)), dim=-1),
        torch.stack((2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)), dim=-1),
        torch.stack((2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)), dim=-1)), dim=-2)


class ArmGravity:
    """The joint torques that hold the D1 still against gravity (`position_only.workspace.gravity_torques`), batched in
    torch, so a hold's command can carry the sag the arm's PD leaves (the insertion's outer loop does on the robot).

    The fingers are folded into Link6 closed; `wrist_payload` adds the RealSense and its bracket (`--payload physical`).
    """

    def __init__(self, kin, wrist_payload: bool = True):
        import numpy as np

        from position_only.workspace import ARM_JOINTS, forward, load_urdf, simulated_coms, simulated_masses

        joints, links = load_urdf()
        masses = simulated_masses(links, wrist_payload=wrist_payload)
        coms = simulated_coms(links, wrist_payload=wrist_payload)
        frames, _, _ = forward(joints, np.zeros((1, 6)))
        r6, p6 = frames["Link6"][0][0], frames["Link6"][1][0]
        m6, m_c = masses["Link6"], masses["Link6"] * coms["Link6"]
        for finger in ("Link7_1", "Link7_2"):
            rf, pf = frames[finger][0][0], frames[finger][1][0]
            in6 = r6.T @ (pf + rf @ coms[finger] - p6)
            m_c = m_c + masses[finger] * in6
            m6 = m6 + masses[finger]
        names = [joints[name]["child"] for name in ARM_JOINTS]
        mass = [masses[name] for name in names[:5]] + [m6]
        com = [coms[name] for name in names[:5]] + [m_c / m6]
        self.kin = kin
        self.mass = torch.as_tensor(mass, dtype=kin.dtype, device=kin.device)
        self.com = torch.as_tensor(np.asarray(com), dtype=kin.dtype, device=kin.device)

    def hold(self, q: torch.Tensor, gravity_b: torch.Tensor | None = None) -> torch.Tensor:
        """(N, 6) joint torques holding `q` still, with gravity `gravity_b` (N, 3, m/s^2) in the base frame (level if None)."""
        n = q.shape[0]
        if gravity_b is None:
            gravity_b = torch.tensor([0.0, 0.0, -9.81], dtype=q.dtype, device=q.device).expand(n, 3)
        axes, origins, coms = [], [], []
        for k in range(6):
            rot, pos = self.kin.chain(q, k + 1)
            axes.append(rot @ self.kin.axis[k].to(q.dtype))
            origins.append(pos)
            coms.append(pos + rot @ self.com[k])
        torque = torch.zeros(n, 6, dtype=q.dtype, device=q.device)
        for j in range(6):
            for link in range(j, 6):
                force = self.mass[link] * gravity_b
                torque[:, j] -= (axes[j] * torch.cross(coms[link] - origins[j], force, dim=-1)).sum(-1)
        return torque

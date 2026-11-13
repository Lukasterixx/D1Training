"""The D1's kinematics, batched in torch: forward, and inverse through its spherical wrist.

`position_only.workspace.forward` is the numpy original and stays the reference (`tests/test_arm_kin.py` holds this to
it). This is the same chain as tensors, so it runs on the GPU beside a training environment, for two jobs the numpy
one cannot do per step: drawing a pose goal that is reachable by construction (`pose_goals.py`) and solving the wrist
for a commanded hand orientation.

**The wrist is spherical.** Read off `d1_arm/d1.urdf`: Joint4's and Joint5's axes meet exactly, and Joint6's passes
1.1 mm from that point (`WRIST_AXIS_MISS_M`, checked). So where the hand *points* is Joint4-6's business and where the
wrist *is* depends on Joint1-3 alone, and a pose splits into two small problems:

    wrist centre  = hand point - R_hand d          three equations in Joint1-3
    R_3^T R_hand  = Rot(Joint4) Rot(Joint5) Rot(Joint6)    an Euler extraction

`d` is the wrist centre to the controlled point, 18.6 cm along the approach. That one number is why F-099 found so few
orientations at a goal: a hand pointing back toward the robot needs its wrist 18.6 cm further out than the goal, and
the arm's reach ends at about 46 cm from the shoulder to the wrist.

Everything is in the Go2 base frame with the mount already in it, as `d1_ik` has it. Angles in radians, URDF joint
order. No Isaac imports.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import torch

from position_only.workspace import ARM_JOINTS, MOUNT_B, SOFT_LIMIT_FACTOR, load_urdf

ROOT = Path(__file__).resolve().parent.parent
URDF = ROOT / "d1_arm/d1.urdf"

#: The controlled point in Link6, `task_cfg.TOOL_OFFSET_M`: the jaw centre, on Joint6's axis.
JAW_CENTRE_LINK6_M = (0.0, 0.0, 0.1051)
#: How far Joint6's axis passes from the point where Joint4's and Joint5's meet, m. A statement about the URDF.
WRIST_AXIS_MISS_M = 0.0011


def _axis_rotation(axis: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    """Rotation matrices about a fixed unit `axis` (3,) by `angle` (N,): (N, 3, 3)."""
    x, y, z = (float(v) for v in axis)
    k = torch.tensor([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=angle.dtype, device=angle.device)
    eye = torch.eye(3, dtype=angle.dtype, device=angle.device)
    s, c = torch.sin(angle)[:, None, None], torch.cos(angle)[:, None, None]
    return eye + s * k + (1.0 - c) * (k @ k)


class ArmKinematics:
    """The chain from the Go2's base to Link6, on `device`."""

    def __init__(self, device="cpu", dtype=torch.float32, urdf: Path = URDF):
        joints, _ = load_urdf(str(urdf))
        self.device, self.dtype = torch.device(device), dtype
        as_t = lambda value: torch.as_tensor(np.asarray(value, dtype=float), dtype=dtype, device=self.device)
        self.xyz = [as_t(joints[name]["xyz"]) for name in ARM_JOINTS]
        self.rpy = [as_t(joints[name]["rpy"]) for name in ARM_JOINTS]
        self.axis = [as_t(joints[name]["axis"] / np.linalg.norm(joints[name]["axis"])) for name in ARM_JOINTS]
        self.mount = as_t(MOUNT_B)
        self.hard_lower = as_t([joints[name]["limits"][0] for name in ARM_JOINTS])
        self.hard_upper = as_t([joints[name]["limits"][1] for name in ARM_JOINTS])
        mid, half = 0.5 * (self.hard_lower + self.hard_upper), 0.5 * (self.hard_upper - self.hard_lower)
        self.soft_lower, self.soft_upper = mid - SOFT_LIMIT_FACTOR * half, mid + SOFT_LIMIT_FACTOR * half
        self.wrist_local = self._wrist_in_link4()
        self.jaw_local = as_t(JAW_CENTRE_LINK6_M)
        self.tool_from_wrist = self._tool_from_wrist()

    def _wrist_in_link4(self) -> torch.Tensor:
        """The point Joint4's and Joint5's axes share, in Link4: Joint5's origin, moved along its own axis onto Joint4's."""
        origin = self.xyz[4].clone()
        axis5 = self.rpy[4] @ self.axis[4]
        # Joint4's axis is Link4's own z through its origin; slide along Joint5's axis until x and y are as near it as they go.
        slide = -(origin[:2] @ axis5[:2]) / (axis5[:2] @ axis5[:2]).clamp(min=1e-9)
        return origin + slide * axis5

    def _tool_from_wrist(self) -> torch.Tensor:
        """The jaw centre from the wrist centre, in Link6. Constant to Joint6's 1.1 mm miss; measured at all-zero."""
        zero = torch.zeros(1, 6, dtype=self.dtype, device=self.device)
        rotation, position = self.link6(zero)
        return rotation[0].T @ (position[0] - self.wrist(zero)[0]) + self.jaw_local

    # --- forward ------------------------------------------------------------------------------

    def chain(self, q: torch.Tensor, upto: int = 6) -> tuple[torch.Tensor, torch.Tensor]:
        """(R, p) of link `upto` (1-6) in the base frame, for joint angles `q` (N, >= upto)."""
        n = q.shape[0]
        rotation = torch.eye(3, dtype=q.dtype, device=q.device).expand(n, 3, 3)
        position = self.mount.to(q.dtype).expand(n, 3)
        for k in range(upto):
            position = position + (rotation @ self.xyz[k].to(q.dtype))
            rotation = rotation @ self.rpy[k].to(q.dtype) @ _axis_rotation(self.axis[k].to(q.dtype), q[:, k])
        return rotation, position

    def link6(self, q: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        return self.chain(q, 6)

    def wrist(self, q: torch.Tensor) -> torch.Tensor:
        """The wrist centre, base frame, from Joint1-4 (Joint4 turns about it, so it drops out)."""
        padded = torch.cat((q[:, :3], torch.zeros_like(q[:, :1])), dim=1)
        rotation, position = self.chain(padded, 4)
        return position + rotation @ self.wrist_local.to(q.dtype)

    def tool(self, q: torch.Tensor, offset: torch.Tensor | None = None) -> tuple[torch.Tensor, torch.Tensor]:
        """(R of Link6, the point `offset` in Link6): the jaw centre by default."""
        rotation, position = self.link6(q)
        local = self.jaw_local.to(q.dtype) if offset is None else offset.to(q.dtype)
        return rotation, position + rotation @ local

    # --- inverse ------------------------------------------------------------------------------

    def _wrist_jacobian(self, q3: torch.Tensor) -> torch.Tensor:
        """d(wrist centre) / d(Joint1-3), (N, 3, 3), analytic: each joint's axis crossed with the lever to the wrist."""
        n = q3.shape[0]
        rotation = torch.eye(3, dtype=q3.dtype, device=q3.device).expand(n, 3, 3)
        position = self.mount.to(q3.dtype).expand(n, 3)
        axes, origins = [], []
        for k in range(3):
            position = position + (rotation @ self.xyz[k].to(q3.dtype))
            rotation = rotation @ self.rpy[k].to(q3.dtype)
            axes.append(rotation @ self.axis[k].to(q3.dtype))
            origins.append(position)
            rotation = rotation @ _axis_rotation(self.axis[k].to(q3.dtype), q3[:, k])
        wrist = self.wrist(torch.cat((q3, torch.zeros_like(q3)), dim=1))
        return torch.stack([torch.cross(a, wrist - o, dim=-1) for a, o in zip(axes, origins)], dim=-1)

    def solve_wrist(self, target: torch.Tensor, seed: torch.Tensor, iterations: int = 12) -> tuple[torch.Tensor, torch.Tensor]:
        """Joint1-3 putting the wrist centre on `target` (N, 3), from `seed` (N, 3). Returns (q3, miss in metres)."""
        q3 = seed.clone()
        eye = 1e-4 * torch.eye(3, dtype=q3.dtype, device=q3.device)
        for _ in range(iterations):
            error = target - self.wrist(torch.cat((q3, torch.zeros_like(q3)), dim=1))
            jac = self._wrist_jacobian(q3)
            jt = jac.transpose(1, 2)
            step = torch.linalg.solve(jt @ jac + eye, (jt @ error.unsqueeze(-1))).squeeze(-1)
            q3 = q3 + step.clamp(-0.4, 0.4)
        error = target - self.wrist(torch.cat((q3, torch.zeros_like(q3)), dim=1))
        return q3, error.norm(dim=-1)

    def solve_hand(self, q3: torch.Tensor, rotation: torch.Tensor) -> list[torch.Tensor]:
        """Joint4-6 turning Link6 to `rotation` (N, 3, 3) from the forearm at `q3`: both wrist solutions, each (N, 3).

        Joint4, 5 and 6 are three successive turns about axes fixed in their links. Written in Link4's frame at Joint4 = 0
        they are `A(q4) B A5 (q5) C A6(q6) D`; the constant parts come off the URDF, so the extraction below is read off
        two products rather than assumed to be any one Euler convention.
        """
        padded = torch.cat((q3, torch.zeros_like(q3[:, :1])), dim=1)
        link4, _ = self.chain(padded, 4)                  # Link4 at Joint4 = 0
        rel = link4.transpose(1, 2) @ rotation            # Link6 in that frame = Rz4(q4) F5 Rz5(q5) F6 Rz6(q6)
        f5, f6 = self._rpy_matrix(4), self._rpy_matrix(5)
        a4, a5, a6 = self.axis[3], self.axis[4], self.axis[5]
        # The hand's axis in Link6 (Joint6's own) maps to a fixed vector in Link5; that vector, seen through Joint5 and
        # Joint4, gives both angles. `h` is Joint6's axis in Link4-at-zero coordinates.
        h = rel @ a6.to(rel.dtype)                        # = Rz4 F5 Rz5 (F6 a6)
        v = f6.to(rel.dtype) @ a6.to(rel.dtype)           # Joint6's axis in Link5
        w = f5.to(rel.dtype) @ a5.to(rel.dtype)           # Joint5's axis in Link4
        solutions = []
        # Joint5: the angle between Joint4's axis and the hand's fixes cos(q5) through the two constant vectors.
        # With J4 axis z4 (Link4), J5 axis w perpendicular to it, J6 axis v perpendicular to J5's: h.z4 = (F5 Rz5 v).z4.
        z4 = a4.to(rel.dtype)
        p = f5.to(rel.dtype).T @ z4                       # Joint4's axis in Link5 at Joint5 = 0
        # Rz5 rotates v about a5 in the plane perpendicular to a5: h.z4 = p . Rot(a5, q5) v = cos(q5) (p.v) + sin(q5) (p.(a5 x v))
        pv = torch.dot(p, v)
        pc = torch.dot(p, torch.cross(a5.to(rel.dtype), v, dim=0))
        amplitude = math.hypot(float(pv), float(pc))
        phase = math.atan2(float(pc), float(pv))
        cos_term = ((h @ z4) / amplitude).clamp(-1.0, 1.0)
        spread = torch.acos(cos_term)
        for q5 in (phase + spread, phase - spread):
            q5 = _wrap(q5)
            # Joint4 from h's direction about z4; Joint6 from what is left.
            rot5 = _axis_rotation(a5.to(rel.dtype), q5)
            h0 = (rot5 @ v) @ f5.to(rel.dtype).T           # h at Joint4 = 0
            q4 = _signed_angle(h0, h, z4)
            rot4 = _axis_rotation(a4.to(rel.dtype), q4)
            rest = (rot4 @ f5.to(rel.dtype) @ rot5 @ f6.to(rel.dtype)).transpose(1, 2) @ rel   # = Rz6(q6)
            # Read the turn about a6 from a vector perpendicular to it.
            e = _perpendicular(a6.to(rel.dtype))
            q6 = _signed_angle(e.expand(rest.shape[0], 3), rest @ e, a6.to(rel.dtype))
            solutions.append(torch.stack((q4, q5, q6), dim=-1))
        return solutions

    def _rpy_matrix(self, k: int) -> torch.Tensor:
        return self.rpy[k]

    def track(self, position: torch.Tensor, rotation: torch.Tensor, seed: torch.Tensor, lower: torch.Tensor, upper: torch.Tensor,
              iterations: int = 5, tol_m: float = 3e-3) -> tuple[torch.Tensor, torch.Tensor]:
        """The solution nearest `seed` (N, 6) for jaw-centre poses near it: (q (N, 6), True where it lies inside `lower`-`upper`).

        For following a path a step at a time, where the elbow's branch is the seed's and only the wrist's two need choosing.
        Near the wrist's singularity Joint4 and Joint6 are ill-defined and the nearer branch may still be out of the box, which
        counts against a candidate goal and no more.
        """
        wrist_goal = position - rotation @ self.tool_from_wrist.to(position.dtype)
        q3, miss = self.solve_wrist(wrist_goal, seed[:, :3], iterations=iterations)
        first, second = self.solve_hand(q3, rotation)
        near = (first - seed[:, 3:]).abs().sum(-1) <= (second - seed[:, 3:]).abs().sum(-1)
        q = torch.cat((q3, torch.where(near.unsqueeze(-1), first, second)), dim=-1)
        return q, (miss < tol_m) & ((q >= lower) & (q <= upper)).all(dim=-1)

    def inverse(self, position: torch.Tensor, rotation: torch.Tensor, seeds: torch.Tensor,
                soft: bool = False, tol_m: float = 2e-3) -> tuple[torch.Tensor, torch.Tensor]:
        """Joint angles for jaw-centre poses (`position` (N, 3), `rotation` (N, 3, 3)) from Joint1-3 `seeds` (S, 3).

        Tries every seed and both wrist branches and keeps, per goal, the solution inside the limits nearest the arm's
        middle. Returns (q (N, 6), solved (N,) bool). `tool_from_wrist` is taken as constant, so a solution is polished by
        one more wrist solve against its own Joint6 miss.
        """
        n = position.shape[0]
        lower = self.soft_lower if soft else self.hard_lower
        upper = self.soft_upper if soft else self.hard_upper
        wrist_goal = position - rotation @ self.tool_from_wrist.to(position.dtype)
        best = torch.zeros(n, 6, dtype=position.dtype, device=position.device)
        best_cost = torch.full((n,), math.inf, dtype=position.dtype, device=position.device)
        for seed in seeds:
            q3, miss = self.solve_wrist(wrist_goal, seed.expand(n, 3))
            q3_ok = (miss < tol_m) & ((q3 >= lower[:3]) & (q3 <= upper[:3])).all(dim=-1)
            for q456 in self.solve_hand(q3, rotation):
                q = torch.cat((q3, q456), dim=-1)
                inside = q3_ok & ((q456 >= lower[3:]) & (q456 <= upper[3:])).all(dim=-1)
                r, p = self.tool(q)
                exact = ((p - position).norm(dim=-1) < 4e-3) & (rotation_angle(r, rotation) < 0.03)
                cost = (q * q).sum(-1)
                take = inside & exact & (cost < best_cost)
                best = torch.where(take.unsqueeze(-1), q, best)
                best_cost = torch.where(take, cost, best_cost)
        return best, torch.isfinite(best_cost)


def _wrap(angle: torch.Tensor | float) -> torch.Tensor:
    angle = torch.as_tensor(angle)
    return torch.remainder(angle + math.pi, 2.0 * math.pi) - math.pi


def _signed_angle(a: torch.Tensor, b: torch.Tensor, axis: torch.Tensor) -> torch.Tensor:
    """The turn about `axis` (3,) taking `a` to `b`, (N, 3) each, after projecting both off the axis."""
    axis = axis.expand_as(a)
    a = a - (a * axis).sum(-1, keepdim=True) * axis
    b = b - (b * axis).sum(-1, keepdim=True) * axis
    return torch.atan2((torch.cross(a, b, dim=-1) * axis).sum(-1), (a * b).sum(-1))


def _perpendicular(axis: torch.Tensor) -> torch.Tensor:
    other = torch.zeros_like(axis)
    other[int(torch.argmin(axis.abs()))] = 1.0
    out = torch.cross(axis, other, dim=0)
    return out / out.norm()


def rotation_angle(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """The geodesic angle between rotation matrices (..., 3, 3), radians."""
    trace = (a * b).sum(dim=(-1, -2))
    return torch.acos(((trace - 1.0) * 0.5).clamp(-1.0, 1.0))

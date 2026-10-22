"""Pressing a plate with the front-right foot: where plates can go, the plate as a plant, and the layer that presses it.

The general task Lukas asked for on 2026-10-21 (`foot_press_cfg`): UniFP's goal walk and pushes at the foot, plus flat
plates placed anywhere the foot reaches, facing any way it can push, rigid or sprung, pressed with a commanded profile.
Built as the key and the handles are (`handle.py`): an analytic plant stepped at the physics rate, so a training run
holds thousands of them, and a task layer in the loop that only knows an estimate of the plate and the foot's position.

**Where plates go** (`PlateTable`): a table of plate geometries built once from the FR leg's kinematics
(`fr_foot_base`, `description/go2_d1.urdf`'s lengths, checked against `demos/combiner_key/classic_stance.foot`). Each
row is a contact point the foot sphere's centre reaches with the base level at both of `REACH_HEIGHTS_M`, a normal
facing the FR thigh joint within 60 degrees (some snapped to walls and level surfaces), the foot able to stand off it
and to push 2 cm past it, and a rectangle round it that clears the floor, the trunk, the head and every leg's links as
the robot stands. The table is in the yaw frame of a robot standing at the origin facing +x, from the ground under its
base -- the environment frame of a robot just reset.

**The plate** (`PlateSet`) is a penalty contact on the foot sphere (and on the knee and the head, frictionless): its
face's overlap through a spring -- `k_soft` over the plate's `travel`, then the rigid 8,000 N/m (`handle.py`'s foot
contact) -- with damping bounded by the spring, and a sticking tangential contact: a spring to where the foot touched
down, slipping past mu x normal. A sphere is in contact only over the rectangle, and only from in front of the slab.
The plate never moves (a sprung plate's face is its spring's travel, not a body).

**The layer** (`PressLayer`) runs from the pre-contact point: the reference in along the estimated normal, a light
touch force (force-aware), contact called where the foot stalls, then the profile -- levels ramped and held, a
tangential part, a slide -- the force off, back out to the standoff. Force-aware: the goal at the origin, the force
commanded. Position-only: no force, the goal `force / k` past the origin.

Plain torch and numpy, no Isaac imports (`tests/test_foot_press.py`). Every number is a choice (`foot_press_cfg`).
"""
from __future__ import annotations

import math

import numpy as np
import torch

from . import foot_press_cfg as pc
from . import leg_cfg
from .handle import (CONTACT_C_N_S_PER_M, CONTACT_K_N_PER_M, DONE, FOOT_RADIUS_M, HOLDING, IDLE, KNEE_RADIUS_M,
                     MAX_OVERLAP_M, PRESSING, RELEASING)
from .insertion import FRICTION_V_EPS, unit

# --- the FR leg's kinematics ----------------------------------------------------------------------------------------

HIP_X, HIP_Y, THIGH_Y, LINK_M, FOOT_SPHERE_X = 0.1934, 0.0465, 0.0955, 0.213, -0.002
#: The FR leg's position limits at 0.95 of their range (`leg_cfg.SOFT_DOF_POS_LIMIT_FR`); the thigh's sample stops at
#: 2.2 rad, past which the foot is behind the hip under the trunk.
FR_HIP_RANGE = (-0.995, 0.995)
FR_THIGH_RANGE = (-1.441, 2.2)
FR_CALF_RANGE = (-2.675, -0.885)


def fr_leg_base(hip, thigh, calf):
    """The FR leg's thigh joint, knee (the calf body's origin) and foot sphere centre in the base frame (numpy)."""
    hip, thigh, calf = (np.asarray(v, dtype=np.float64) for v in (hip, thigh, calf))
    c, s = np.cos(hip), np.sin(hip)

    def rolled(x, y, z):
        return np.stack((HIP_X + x, -HIP_Y + c * y - s * z, s * y + c * z), axis=-1)

    y = -THIGH_Y * np.ones_like(thigh)
    zero = np.zeros_like(thigh)
    joint = rolled(zero, y, zero)
    knee = rolled(-LINK_M * np.sin(thigh), y, -LINK_M * np.cos(thigh))
    fx = -LINK_M * np.sin(thigh) - LINK_M * np.sin(thigh + calf) + FOOT_SPHERE_X * np.cos(thigh + calf)
    fz = -LINK_M * np.cos(thigh) - LINK_M * np.cos(thigh + calf) + FOOT_SPHERE_X * np.sin(thigh + calf)
    foot = rolled(fx, y, fz)
    return joint, knee, foot


def leg_points_base(leg: str, q) -> np.ndarray:
    """Hip-joint, knee and foot of any leg (the FR leg mirrored), base frame, (3, 3)."""
    sx = 1.0 if leg[0] == "F" else -1.0
    sy = 1.0 if leg[1] == "L" else -1.0
    joint, knee, foot = fr_leg_base(sy * -q[0], q[1], q[2])     # the left legs' hips roll the other way
    out = np.stack((joint, knee, foot))
    out[:, 0] += (sx - 1.0) * HIP_X                              # the hip's offset mirrors front to back, the leg does not
    out[:, 1] *= -sy
    return out


class ReachVoxels:
    """Where the FR foot sphere's centre reaches with the base level at a height: a 1 cm voxel grid of a kinematic
    sample, dilated by a tolerance. In the yaw frame from the ground under the base."""

    LO = np.array([-0.25, -0.65, -0.05])
    SIZE = np.array([100, 100, 90])
    CELL = 0.01

    def __init__(self, base_height: float, tol: float = pc.REACH_TOL_M, samples=(25, 60, 44)) -> None:
        hip = np.linspace(*FR_HIP_RANGE, samples[0])
        thigh = np.linspace(*FR_THIGH_RANGE, samples[1])
        calf = np.linspace(*FR_CALF_RANGE, samples[2])
        h, t, k = np.meshgrid(hip, thigh, calf, indexing="ij")
        _, _, foot = fr_leg_base(h.ravel(), t.ravel(), k.ravel())
        foot[:, 2] += base_height
        self.points = foot[~_in_box(foot, pc.TRUNK_KEEPOUT_M)]
        grid = np.zeros(self.SIZE, dtype=bool)
        idx = np.floor((self.points - self.LO) / self.CELL).astype(int)
        ok = np.all((idx >= 0) & (idx < self.SIZE), axis=1)
        grid[tuple(idx[ok].T)] = True
        reach = int(math.ceil(tol / self.CELL))
        out = np.zeros_like(grid)
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                for dz in range(-reach, reach + 1):
                    if dx * dx + dy * dy + dz * dz > (tol / self.CELL) ** 2 + 1e-9:
                        continue
                    out |= np.roll(grid, (dx, dy, dz), axis=(0, 1, 2))
        self.grid = out

    def __call__(self, p: np.ndarray) -> np.ndarray:
        idx = np.floor((np.asarray(p) - self.LO) / self.CELL).astype(int)
        ok = np.all((idx >= 0) & (idx < self.SIZE), axis=-1)
        out = np.zeros(idx.shape[:-1], dtype=bool)
        out[ok] = self.grid[tuple(idx[ok].T)]
        return out


def _in_box(p: np.ndarray, box) -> np.ndarray:
    (x0, x1), (y0, y1), (z0, z1) = box
    return (p[..., 0] > x0) & (p[..., 0] < x1) & (p[..., 1] > y0) & (p[..., 1] < y1) & (p[..., 2] > z0) & (p[..., 2] < z1)


def _body_points(base_height: float = leg_cfg.SETTLED_BASE_HEIGHT_M, per_link: int = 8) -> np.ndarray:
    """Points along every leg's links (hip joint -> knee -> foot) at the settled stance, and through the trunk keep-out
    and the head, yaw frame from the ground: what a plate must keep clear of as the robot stands."""
    q = np.array(leg_cfg.SETTLED_LEG_POS).reshape(4, 3)
    pts = []
    s = np.linspace(0.0, 1.0, per_link)[:, None]
    for k, leg in enumerate(("FL", "FR", "RL", "RR")):
        joint, knee, foot = leg_points_base(leg, q[k])
        pts += [joint + s * (knee - joint), knee + s * (foot - knee)]
    (x0, x1), (y0, y1), (z0, z1) = pc.TRUNK_KEEPOUT_M
    gx, gy, gz = np.meshgrid(np.linspace(x0, x1, 8), np.linspace(y0, y1, 5), np.linspace(z0, z1, 5), indexing="ij")
    trunk = np.stack((gx.ravel(), gy.ravel(), gz.ravel() - base_height), axis=-1)
    out = np.concatenate(pts + [trunk])
    out[:, 2] += base_height
    return out


def _basis(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors spanning the plane normal to each `n`."""
    helper = np.where(np.abs(n[:, 2:3]) < 0.9, np.array([[0.0, 0.0, 1.0]]), np.array([[1.0, 0.0, 0.0]]))
    e1 = np.cross(helper, n)
    e1 /= np.linalg.norm(e1, axis=-1, keepdims=True)
    return e1, np.cross(n, e1)


class PlateTable:
    """Plate geometries the foot can press (see the module docstring), yaw frame from the ground under the base.

    Columns (numpy, then torch with `to`): `contact` the foot sphere's centre touching at the contact point, `face` the
    face's centre, `n` its outward normal, `t1`, `t2` its in-plane axes, `half` its half-sizes along them, `kind` 0
    tilted, 1 wall, 2 level facing up, 3 level facing down."""

    def __init__(self, size: int = pc.TABLE_SIZE, seed: int = pc.TABLE_SEED) -> None:
        rng = np.random.default_rng(seed)
        reach = [ReachVoxels(h) for h in pc.REACH_HEIGHTS_M]
        body = _body_points()
        lo, hi = np.array(pc.CONTACT_BOX_M)[:, 0], np.array(pc.CONTACT_BOX_M)[:, 1]
        shoulder = np.array(pc.SHOULDER_M)
        kept = {k: [] for k in ("contact", "face", "n", "t1", "t2", "half", "kind")}
        total, tries = 0, 0
        while total < size and tries < 200:
            tries += 1
            m = 4 * size
            # Uniform in space, within the comfortable part of the leg's reach (until 2026-10-21 22:50 the points were
            # drawn from the kinematic sample itself: uniform in joint space, they sat at the reach's outer shell --
            # median 0.31 m from the thigh joint, 0.37 at p90 -- and the foot ended its approaches ~20 cm short).
            p = lo + rng.uniform(0, 1, (m, 3)) * (hi - lo)
            # Half of them ahead of the foot, where most things to press are (the rest would put two in three to the
            # side: the trunk's keep-out and the standing leg reject more plates ahead).
            ahead = rng.uniform(0, 1, m) < pc.AHEAD_SHARE
            flo, fhi = np.array(pc.AHEAD_BOX_M)[:, 0], np.array(pc.AHEAD_BOX_M)[:, 1]
            p[ahead] = flo + rng.uniform(0, 1, (int(ahead.sum()), 3)) * (fhi - flo)
            dist = np.linalg.norm(p - shoulder, axis=-1)
            ok = (dist >= pc.CONTACT_DIST_M[0]) & (dist <= pc.CONTACT_DIST_M[1])
            ok &= (p[:, 2] >= pc.MIN_CONTACT_HEIGHT_M) & (p[:, 0] >= leg_cfg.GOAL_MIN_AHEAD_M) \
                & (p[:, 1] <= leg_cfg.GOAL_MAX_LEFT_M)
            for r in reach:
                ok &= r(p)
            d0 = shoulder - p
            d0 /= np.linalg.norm(d0, axis=-1, keepdims=True)
            cos_t = 1.0 - rng.uniform(0, 1, m) * (1.0 - math.cos(pc.NORMAL_TILT_MAX_RAD))
            sin_t = np.sqrt(np.clip(1.0 - cos_t ** 2, 0.0, 1.0))
            phi = rng.uniform(0, 2 * math.pi, m)
            e1, e2 = _basis(d0)
            n = cos_t[:, None] * d0 + sin_t[:, None] * (np.cos(phi)[:, None] * e1 + np.sin(phi)[:, None] * e2)
            kind = np.zeros(m, dtype=np.int64)
            u = rng.uniform(0, 1, m)
            wall = u < pc.WALL_SNAP_PROB
            level = (u >= pc.WALL_SNAP_PROB) & (u < pc.WALL_SNAP_PROB + pc.LEVEL_SNAP_PROB)
            flat = n.copy()
            flat[:, 2] = 0.0
            wall &= np.linalg.norm(flat, axis=-1) > 0.3
            n[wall] = flat[wall] / np.linalg.norm(flat[wall], axis=-1, keepdims=True)
            kind[wall] = 1
            up = np.sign(n[:, 2]) >= 0
            n[level] = np.where(up[level, None], [[0.0, 0.0, 1.0]], [[0.0, 0.0, -1.0]])
            kind[level] = np.where(up[level], 2, 3)
            for r in reach:
                ok &= r(p - pc.PUSH_ROOM_M * n) & r(p + pc.STANDOFF_REACH_M * n)
            # The waypoint too (the approach comes in along the normal from it), out of the trunk.
            way = p + (pc.STANDOFF_REACH_M + pc.WAYPOINT_OUT_M) * n
            ok &= ~_in_box(way, pc.TRUNK_KEEPOUT_M) & (np.linalg.norm(way - shoulder, axis=-1) <= pc.CONTACT_DIST_M[1] + 0.03) \
                & (way[:, 2] >= pc.MIN_CONTACT_HEIGHT_M)
            half = rng.uniform(*pc.PLATE_HALF_M, (m, 2))
            b1, b2 = _basis(n)
            psi = rng.uniform(0, math.pi, m)[:, None]
            t1 = np.cos(psi) * b1 + np.sin(psi) * b2
            t2 = np.cross(n, t1)
            offset = rng.uniform(-pc.CONTACT_OFFSET_SHARE, pc.CONTACT_OFFSET_SHARE, (m, 2)) * half
            face = p - FOOT_RADIUS_M * n - offset[:, :1] * t1 - offset[:, 1:] * t2
            # The rectangle above the floor, out of the trunk, and clear of the legs and the body as the robot stands.
            grid = np.linspace(-1.0, 1.0, 5)
            for a in grid:
                for b in grid:
                    corner = face + (a * half[:, :1]) * t1 + (b * half[:, 1:]) * t2
                    ok &= corner[:, 2] >= 0.0
                    ok &= ~_in_box(corner, pc.TRUNK_KEEPOUT_M)
            clr = pc.LEG_CLEARANCE_M
            idx = np.flatnonzero(ok)
            for chunk in np.array_split(idx, max(1, len(idx) // 4096)):
                rel = body[None, :, :] - face[chunk, None, :]
                h = (rel * n[chunk, None, :]).sum(-1)
                a = np.abs((rel * t1[chunk, None, :]).sum(-1))
                b = np.abs((rel * t2[chunk, None, :]).sum(-1))
                hit = (a < half[chunk, None, 0] + clr) & (b < half[chunk, None, 1] + clr) \
                    & (h > -pc.PLATE_THICKNESS_M - clr) & (h < clr)
                ok[chunk] &= ~hit.any(axis=1)
            for key, value in (("contact", p), ("face", face), ("n", n), ("t1", t1), ("t2", t2), ("half", half),
                               ("kind", kind)):
                kept[key].append(value[ok])
            total += int(ok.sum())
        self.columns = {k: np.concatenate(v)[:size] for k, v in kept.items()}
        self.size = len(self.columns["kind"])

    def to(self, device) -> dict[str, torch.Tensor]:
        return {k: torch.as_tensor(v, dtype=torch.long if k == "kind" else torch.float32, device=device)
                for k, v in self.columns.items()}


# --- the plate -------------------------------------------------------------------------------------------------------

class PlateSet:
    """A batch of plates, one per environment; inactive until `place`d. Attribute names the leg environment reads off
    its handle are kept (`on_bar`, `touched`, `door_n`, `knee_door_n`, `force_on_foot`, `knee_force`, `head_force`)."""

    def __init__(self, num_envs: int, device, *, k: float = CONTACT_K_N_PER_M, c: float = CONTACT_C_N_S_PER_M,
                 edges: bool = True) -> None:
        self.n_envs, self.device, self.k, self.c = num_envs, torch.device(device), k, c
        #: The plate as a slab with edges and a back (`_normal`); False: its face only, as trained until 2026-10-22 10:55.
        self.edges = edges
        zeros = lambda *shape: torch.zeros(*shape, device=self.device)
        flags = lambda: torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.active = flags()
        self.face, self.n, self.t1, self.t2 = zeros(num_envs, 3), zeros(num_envs, 3), zeros(num_envs, 3), zeros(num_envs, 3)
        self.n[:, 0], self.t1[:, 1], self.t2[:, 2] = -1.0, 1.0, 1.0
        self.half = zeros(num_envs, 2)
        self.k_soft, self.travel, self.mu = torch.full((num_envs,), k, device=self.device), zeros(num_envs), zeros(num_envs)
        self.kind = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.anchor, self.anchored = zeros(num_envs, 3), flags()
        self.force_on_foot, self.knee_force, self.head_force = zeros(num_envs, 3), zeros(num_envs, 3), zeros(num_envs, 3)
        #: The foot's mean normal force and overlap over the last step; the knee's and head's normal forces.
        self.normal_n, self.depth = zeros(num_envs), zeros(num_envs)
        self.knee_n, self.head_n = zeros(num_envs), zeros(num_envs)
        self.door_n, self.knee_door_n = zeros(num_envs), zeros(num_envs)
        self.on_bar, self.touched = flags(), flags()
        #: The foot centre's height off the face, last step (positive in front).
        self.height = zeros(num_envs)
        self._peak = zeros(num_envs)

    def place(self, ids: torch.Tensor, *, face: torch.Tensor, n: torch.Tensor, t1: torch.Tensor, half: torch.Tensor,
              k_soft: torch.Tensor, travel: torch.Tensor, mu: torch.Tensor, kind: torch.Tensor | None = None) -> None:
        n = unit(n)
        t1 = unit(t1 - (t1 * n).sum(-1, keepdim=True) * n)
        self.face[ids], self.n[ids], self.t1[ids], self.t2[ids] = face, n, t1, torch.cross(n, t1, dim=-1)
        self.half[ids], self.k_soft[ids], self.travel[ids], self.mu[ids] = half, k_soft, travel, mu
        if kind is not None:
            self.kind[ids] = kind
        self.anchored[ids], self.touched[ids], self.on_bar[ids] = False, False, False
        self.active[ids] = True

    def reset(self, ids: torch.Tensor) -> None:
        self.active[ids] = False
        for buf in (self.force_on_foot, self.knee_force, self.head_force, self.normal_n, self.depth, self.knee_n,
                    self.head_n, self.door_n, self.knee_door_n, self._peak, self.height):
            buf[ids] = 0.0
        self.anchored[ids], self.touched[ids], self.on_bar[ids] = False, False, False

    def take_peak(self) -> torch.Tensor:
        peak = self._peak.clone()
        self._peak.zero_()
        return peak

    def _normal(self, p: torch.Tensor, v: torch.Tensor, r: float):
        """The plate's force on a sphere at `p` moving at `v`, along its contact normal: (magnitude, touching, the
        normal (world, out of the plate), overlap).

        With `edges` (since 2026-10-22 10:55) the plate is the slab it is drawn as, `PLATE_THICKNESS_M` behind its face,
        and the sphere meets its nearest point -- the face, an edge or the back -- as the key's platform does
        (`handle.LeverHandle._pad_contact`); only the face has the sprung plate's travel. Without them (rounds 1-5 and
        the replication) only the face pushes, along `n`, and only over the rectangle: a sphere coming in from the side
        at the face's height met nothing until its centre crossed the edge, then the whole of its radius at once (the
        replication's final: 14 of its 21 damaged presses, the foot returning past a plate facing down)."""
        rel = p - self.face
        u, w, h = (rel * self.t1).sum(-1), (rel * self.t2).sum(-1), (rel * self.n).sum(-1)
        if self.edges:
            local = torch.stack((u, w, h), dim=-1)
            lo = torch.stack((-self.half[:, 0], -self.half[:, 1], torch.full_like(h, -pc.PLATE_THICKNESS_M)), dim=-1)
            hi = torch.stack((self.half[:, 0], self.half[:, 1], torch.zeros_like(h)), dim=-1)
            q = torch.maximum(torch.minimum(local, hi), lo)
            gap = local - q
            dist = gap.norm(dim=-1)
            inside = dist < 1e-9
            normal_l = torch.where(inside.unsqueeze(-1), torch.tensor([0.0, 0.0, 1.0], device=p.device).expand_as(gap),
                                   gap / dist.clamp(min=1e-9).unsqueeze(-1))
            overlap = torch.where(inside, r - h, r - dist).clamp(min=0.0)
            normal = normal_l[:, :1] * self.t1 + normal_l[:, 1:2] * self.t2 + normal_l[:, 2:] * self.n
            face = normal_l[:, 2] > 0.7
            touching = self.active & (overlap > 0.0)
        else:
            inside = (u.abs() <= self.half[:, 0]) & (w.abs() <= self.half[:, 1])
            overlap = (r - h).clamp(min=0.0)
            touching = self.active & inside & (h > -pc.PLATE_THICKNESS_M) & (overlap > 0.0)
            normal, face = self.n, torch.ones_like(touching)
        soft = self.k_soft * torch.minimum(overlap, self.travel)
        hard = self.k * (overlap - self.travel).clamp(min=0.0, max=MAX_OVERLAP_M)
        spring = torch.where(face, soft + hard, self.k * overlap.clamp(max=MAX_OVERLAP_M))
        closing = -(v * normal).sum(-1)
        f_n = (spring + (self.c * closing).clamp(min=-spring, max=spring)).clamp(min=0.0) * touching
        return f_n, touching, normal, overlap * touching

    def _slide_friction(self, f_n: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
        slide = v - (v * self.n).sum(-1, keepdim=True) * self.n
        speed = slide.norm(dim=-1)
        return (-(self.mu * f_n).unsqueeze(-1) * slide / speed.clamp(min=1e-9).unsqueeze(-1)
                * torch.tanh(speed / FRICTION_V_EPS).unsqueeze(-1))

    def step(self, foot: torch.Tensor, foot_v: torch.Tensor, dt: float, *, foot_r: float = FOOT_RADIUS_M,
             knee: torch.Tensor | None = None, knee_v: torch.Tensor | None = None,
             head: torch.Tensor | None = None, head_v: torch.Tensor | None = None, head_r: float = 0.065) -> torch.Tensor:
        """Advance one physics step; return the mean force on the foot (world). The knee and the head, if given, are
        pressed off the plate's face too (`knee_force`, `head_force`), without friction."""
        h = dt / pc.SUBSTEPS
        total = torch.zeros_like(foot)
        n_sum, d_sum = torch.zeros_like(self.normal_n), torch.zeros_like(self.depth)
        any_touch = torch.zeros_like(self.active)
        for k in range(pc.SUBSTEPS):
            here = foot + foot_v * ((k + 0.5) * h)
            f_n, touching, normal, overlap = self._normal(here, foot_v, foot_r)
            # Stick to where it touched down; slip past mu x normal, the anchor dragged along.
            fresh = touching & ~self.anchored
            self.anchor = torch.where(fresh.unsqueeze(-1), here, self.anchor)
            self.anchored = touching
            delta = here - self.anchor
            delta = delta - (delta * normal).sum(-1, keepdim=True) * normal
            slide = foot_v - (foot_v * normal).sum(-1, keepdim=True) * normal
            spring_t = -pc.TANGENT_K_N_PER_M * delta
            damp_t = -pc.TANGENT_C_N_S_PER_M * slide
            cap = (self.mu * f_n).unsqueeze(-1)
            damp_t = damp_t * (cap / damp_t.norm(dim=-1, keepdim=True).clamp(min=1e-9)).clamp(max=1.0)
            f_t = spring_t + damp_t
            mag = f_t.norm(dim=-1, keepdim=True)
            slipping = mag > cap
            f_t = torch.where(slipping, f_t * cap / mag.clamp(min=1e-9), f_t)
            # Slipping: the anchor follows so the spring sits at the friction limit.
            spring_cap = torch.where(slipping, -pc.TANGENT_K_N_PER_M * delta * (cap / mag.clamp(min=1e-9)), spring_t)
            self.anchor = torch.where(slipping & touching.unsqueeze(-1), here + spring_cap / pc.TANGENT_K_N_PER_M, self.anchor)
            f_t = f_t * touching.unsqueeze(-1)
            on_foot = f_n.unsqueeze(-1) * normal + f_t
            total += on_foot
            n_sum += f_n
            d_sum += overlap
            any_touch |= touching
            self._peak = torch.maximum(self._peak, on_foot.norm(dim=-1))
        self.force_on_foot = total / pc.SUBSTEPS
        self.normal_n, self.depth = n_sum / pc.SUBSTEPS, d_sum / pc.SUBSTEPS
        self.height = ((foot - self.face) * self.n).sum(-1)
        self.on_bar = any_touch
        self.touched |= any_touch
        self.knee_force = torch.zeros_like(foot)
        self.knee_n = torch.zeros_like(self.normal_n)
        if knee is not None:
            f_n, _, normal, _ = self._normal(knee, knee_v, KNEE_RADIUS_M)
            self.knee_force, self.knee_n = f_n.unsqueeze(-1) * normal, f_n
            self._peak = torch.maximum(self._peak, f_n)
        self.head_force = torch.zeros_like(foot)
        self.head_n = torch.zeros_like(self.normal_n)
        if head is not None:
            f_n, _, normal, _ = self._normal(head, head_v, head_r)
            self.head_force, self.head_n = f_n.unsqueeze(-1) * normal, f_n
        self.knee_door_n = self.knee_n + self.head_n
        return self.force_on_foot


# --- the layer ---------------------------------------------------------------------------------------------------------

#: Most segments a profile has.
MAX_SEGMENTS = 3


class PressLayer:
    """The press as a robot would run it from an estimate of the plate, reading only the foot's position: in along the
    estimated normal until the foot stalls, then a profile of forces, then back out. See the module docstring."""

    def __init__(self, num_envs: int, device, *, k_virtual: float = leg_cfg.FOOT_FORCE_KP,
                 release_from_foot: bool = True) -> None:
        self.n_envs, self.device, self.k_virtual = num_envs, torch.device(device), k_virtual
        #: Start the release from the foot, wherever it is (`step`); False: from the press point, as trained until 10:45.
        self.release_from_foot = release_from_foot
        zeros = lambda *shape: torch.zeros(*shape, device=self.device)
        flags = lambda: torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.phase = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.pre, self.n, self.origin = zeros(num_envs, 3), zeros(num_envs, 3), zeros(num_envs, 3)
        self.n[:, 0] = -1.0
        for name in ("standoff", "speed", "ref", "timer", "seg_t", "wait", "level", "slide_speed", "slide_dist",
                     "slide_s", "release_from", "depth", "start_level"):
            setattr(self, name, zeros(num_envs))
        self.levels, self.ramps, self.holds, self.tan_ratio = (zeros(num_envs, MAX_SEGMENTS) for _ in range(4))
        self.tan_dir, self.slide_dir = zeros(num_envs, 3), zeros(num_envs, 3)
        self.segments = torch.ones(num_envs, dtype=torch.long, device=self.device)
        self.seg = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.slide, self.force_law = flags(), flags()
        #: True once contact was called by the foot stalling (False if the reference ran out first).
        self.stalled = flags()
        self.last_foot = zeros(num_envs, 3)
        self.goal, self.force_cmd = zeros(num_envs, 3), zeros(num_envs, 3)
        #: The way the foot pushes (the estimated inward normal), for the outer loop to leave alone.
        self.tangent = zeros(num_envs, 3)
        self.tangent[:, 0] = 1.0
        #: The force the profile asks for now (N, normal; the tangential part on top), for both variants.
        self.normal_level = zeros(num_envs)

    def start(self, ids: torch.Tensor, *, pre: torch.Tensor, n: torch.Tensor, standoff: torch.Tensor,
              speed: torch.Tensor, levels: torch.Tensor, ramps: torch.Tensor, holds: torch.Tensor,
              segments: torch.Tensor, tan_ratio: torch.Tensor, tan_dir: torch.Tensor, slide: torch.Tensor,
              slide_dir: torch.Tensor, slide_speed: torch.Tensor, slide_dist: torch.Tensor, force_law: torch.Tensor,
              foot: torch.Tensor) -> None:
        """Begin for `ids` from the pre-contact point `pre`, `standoff` off the estimated face along its normal `n`."""
        self.pre[ids], self.n[ids] = pre, unit(n)
        self.standoff[ids], self.speed[ids] = standoff, speed
        self.levels[ids], self.ramps[ids], self.holds[ids], self.segments[ids] = levels, ramps, holds, segments
        self.tan_ratio[ids], self.tan_dir[ids] = tan_ratio, tan_dir
        self.slide[ids], self.slide_dir[ids] = slide, slide_dir
        self.slide_speed[ids], self.slide_dist[ids] = slide_speed, slide_dist
        self.force_law[ids] = force_law
        self.phase[ids] = PRESSING
        for buf in (self.ref, self.timer, self.seg_t, self.wait, self.level, self.slide_s, self.normal_level,
                    self.release_from, self.depth, self.start_level):
            buf[ids] = 0.0
        self.seg[ids] = 0
        self.stalled[ids] = False
        self.origin[ids] = pre
        self.last_foot[ids] = foot
        self.goal[ids] = pre
        self.force_cmd[ids] = 0.0
        self.tangent[ids] = -self.n[ids]

    def reset(self, ids: torch.Tensor) -> None:
        self.phase[ids] = IDLE
        for buf in (self.ref, self.timer, self.seg_t, self.wait, self.level, self.slide_s, self.normal_level,
                    self.release_from, self.depth, self.start_level):
            buf[ids] = 0.0
        self.seg[ids] = 0
        self.stalled[ids] = False
        self.goal[ids], self.force_cmd[ids] = 0.0, 0.0

    def _segment(self, values: torch.Tensor) -> torch.Tensor:
        """Each environment's value for its current segment (its last one once the profile is through)."""
        index = torch.minimum(self.seg, self.segments - 1).clamp(min=0)
        return values.gather(1, index.unsqueeze(-1)).squeeze(-1)

    def step(self, foot: torch.Tensor, dt: float) -> tuple[torch.Tensor, torch.Tensor]:
        """Advance one policy step; return (the goal for the foot's centre, the force command), both world."""
        speed_in = ((foot - self.last_foot) * -self.n).sum(-1) / dt
        self.last_foot = foot.clone()
        # In along the normal until the foot stalls.
        approaching = self.phase == PRESSING
        end = self.standoff + pc.OVER_TRAVEL_M
        self.ref = torch.where(approaching, torch.minimum(self.ref + self.speed * dt, end), self.ref)
        reference = self.pre - self.ref.unsqueeze(-1) * self.n
        miss = reference - foot
        lag = (miss * -self.n).sum(-1)
        # A stall counts only near where the face is expected and with the foot tracking the reference across the
        # normal: a foot still centimetres off its line lags the reference without touching anything (the first round
        # of 2026-10-21, engaged at the training deadline 12 cm off, called contact in the air in every episode).
        across_miss = (miss + lag.unsqueeze(-1) * self.n).norm(dim=-1)
        near = self.ref >= self.standoff - pc.DETECT_FROM_M
        stalled = approaching & near & (across_miss < pc.DETECT_ACROSS_M) & (lag > pc.DETECT_LAG_M) \
            & (speed_in < pc.DETECT_SPEED_M_S)
        self.wait = torch.where(approaching & (self.ref >= end), self.wait + dt, self.wait)
        ran_out = approaching & (self.wait >= pc.DETECT_WAIT_S)
        found = stalled | ran_out
        self.stalled |= stalled
        # Where the foot stalled is the press's origin (its in-plane part from the reference, not the foot's drift).
        across = reference + ((foot - reference) * self.n).sum(-1, keepdim=True) * self.n
        self.origin = torch.where(found.unsqueeze(-1), across, self.origin)
        # The first ramp starts from the force the target already asked for -- the touch plus the impedance on the
        # foot's lag behind the reference -- so the target does not jump back to the foot when contact is called (it
        # did, by the lag, until 2026-10-22 01:05: round 3's model_4000 lost the plate for over 0.1 s in 116 of 288
        # presses, most at the start of the ramp). Exact for both variants: the force-aware target is the origin plus
        # the force over k, the position-only goal the same point.
        implied = (pc.TOUCH_N + self.k_virtual * lag.clamp(min=0.0)).clamp(max=pc.FORCE_LEVELS_N[1])
        self.start_level = torch.where(found, implied, self.start_level)
        self.phase = torch.where(found, torch.full_like(self.phase, HOLDING), self.phase)
        self.seg = torch.where(found, torch.zeros_like(self.seg), self.seg)
        self.seg_t = torch.where(found, torch.zeros_like(self.seg_t), self.seg_t)
        # The profile: each segment ramps from the last level to its own and holds it.
        holding = self.phase == HOLDING
        self.seg_t = torch.where(holding & ~found, self.seg_t + dt, self.seg_t)
        ramp, hold = self._segment(self.ramps).clamp(min=dt), self._segment(self.holds)
        prev = torch.where(self.seg > 0, self.levels.gather(1, (self.seg - 1).clamp(min=0).unsqueeze(-1)).squeeze(-1),
                           self.start_level)
        target = self._segment(self.levels)
        level = prev + (target - prev) * (self.seg_t / ramp).clamp(max=1.0)
        self.level = torch.where(holding, level, self.level)
        last = self.seg == self.segments - 1
        sliding = holding & last & self.slide & (self.seg_t >= ramp)
        self.slide_s = torch.where(sliding, torch.minimum(self.slide_s + self.slide_speed * dt, self.slide_dist), self.slide_s)
        next_seg = holding & (self.seg_t >= ramp + hold)
        self.seg = torch.where(next_seg, self.seg + 1, self.seg)
        self.seg_t = torch.where(next_seg, torch.zeros_like(self.seg_t), self.seg_t)
        finished = next_seg & (self.seg >= self.segments)
        self.release_from = torch.where(finished, self.level, self.release_from)
        if self.release_from_foot:
            # The release starts from where the foot is and backs straight out along the normal: it never draws the
            # foot back to the press point. A foot that slid off a narrow plate's edge under a large push (pushing up on
            # a 6 cm strip at 28 N, 17 cm off by the end of the hold) was pulled back under it and met the slab at
            # 126-150 N (2026-10-22 10:40, the replication's final, plates facing down). For a press that held, the foot
            # is at the press point and nothing changes.
            self.origin = torch.where(finished.unsqueeze(-1), foot - self.slide_s.unsqueeze(-1) * self.slide_dir
                                      + self.depth.unsqueeze(-1) * self.n, self.origin)
        self.phase = torch.where(finished, torch.full_like(self.phase, RELEASING), self.phase)
        self.timer = torch.where(finished, torch.zeros_like(self.timer), self.timer)
        # The release: the force off, then back out to the standoff.
        releasing = self.phase == RELEASING
        self.timer = torch.where(releasing & ~finished, self.timer + dt, self.timer)
        off = (1.0 - self.timer / pc.RELEASE_FORCE_S).clamp(min=0.0)
        self.level = torch.where(releasing, self.release_from * off, self.level)
        back = ((self.timer - pc.RELEASE_FORCE_S) / pc.RELEASE_S).clamp(0.0, 1.0)
        done = releasing & (self.timer >= pc.RELEASE_FORCE_S + pc.RELEASE_S)
        self.phase = torch.where(done, torch.full_like(self.phase, DONE), self.phase)

        # What the foot is asked for now, by the phase each environment ends this step in.
        approaching, holding, releasing = self.phase == PRESSING, self.phase == HOLDING, self.phase == RELEASING
        # The force-aware layer's goal follows the foot along the normal while it presses (bounded), so the target
        # UniFP scores, goal + (external + command) / k, is met only where the force on the plate is the command,
        # whatever the plate's stiffness. Held at the origin, the goal made the target a spring of k: a sprung plate
        # of stiffness k_s then settles at k_s / (k_s + k) of the command (71% at 1,000 N/m).
        sunk = ((self.origin - foot) * self.n).sum(-1)
        follow = holding & self.force_law & ~found
        depth = self.depth + (sunk - self.depth) * min(1.0, dt / pc.FOLLOW_TAU_S)
        self.depth = torch.where(follow, depth.clamp(-pc.FOLLOW_OUT_M, pc.FOLLOW_MAX_M), self.depth)
        pressed = self.origin + self.slide_s.unsqueeze(-1) * self.slide_dir - self.depth.unsqueeze(-1) * self.n
        out = pressed + (self.standoff * back).unsqueeze(-1) * self.n
        tan = (self.level * self._segment(self.tan_ratio)).unsqueeze(-1) * self.tan_dir
        level = torch.where(holding | releasing, self.level, torch.zeros_like(self.level))
        self.normal_level = level
        force = -level.unsqueeze(-1) * self.n + tan * (holding | releasing).unsqueeze(-1)
        touch = -pc.TOUCH_N * self.n
        force = torch.where(approaching.unsqueeze(-1), touch, force)
        force = torch.where(((self.phase == IDLE) | (self.phase == DONE)).unsqueeze(-1), torch.zeros_like(force), force)
        law = self.force_law.unsqueeze(-1)
        self.force_cmd = torch.where(law, force, torch.zeros_like(force))
        # The position-only layer has no force to command: it asks for the target the force would make.
        displaced = torch.where(law, torch.zeros_like(force), force / self.k_virtual)
        goal = torch.where(approaching.unsqueeze(-1), reference, out) + displaced
        goal = torch.where((self.phase == DONE).unsqueeze(-1), pressed + self.standoff.unsqueeze(-1) * self.n, goal)
        active = self.phase != IDLE
        self.goal = torch.where(active.unsqueeze(-1), goal, self.goal)
        return self.goal, self.force_cmd

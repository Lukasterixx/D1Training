"""Turning a lever door handle down with a foot: the plant, and the task layer that drives the turn.

Lukas's question of 2026-10-19: the Go2's front-right leg is far stronger than the D1 (23.7 / 23.7 / 45.4 N·m at the
hip, thigh and knee against the D1's 3.33 N·m shoulder), so can it turn a door handle at the Go2's own body height,
with the arm holding the wrist camera on what the foot is doing? The door does not have to open: the task ends when
the latch has been held retracted. Built as the other contact tasks are (`pressing.py`, `insertion.py`): an analytic
plant stepped at the physics rate, so a training run can hold thousands of them, and a task layer that drives the turn
from an estimate of where the handle is.

**The handle** is a lever on a spindle through a door, in the environment frame:

  * the door's face is the plane through `door` with unit normal `n`, horizontal, out toward the robot;
  * the spindle's axis passes through `pivot = door + standoff n`, the lever's centreline `standoff` proud of the face;
  * at rest the lever points along `u0`, horizontal and in the door's plane; it turns *down*: at angle `theta` it
    points along `u(theta) = cos(theta) u0 - sin(theta) z`, about the axis `a = z x u0` (which is +-n);
  * the bar is a capsule of radius `radius` from `pivot + inner u` to `pivot + length u` (the inner part is the rose);
  * it resists with `preload + stiffness theta` (a return spring holding it against a rest stop at 0), viscous
    `damping`, a Coulomb `friction` in the spindle and an end stop at `theta_max`; the latch is retracted while
    `theta >= theta_latch`. Its inertia is the lever's plus the latch mechanism's, and it is integrated with
    `SUBSTEPS` sub-steps per physics step, the foot carried along its velocity, because the contact's rotational
    stiffness (16,000 N/m at 0.1 m: 160 N·m/rad) on a 1e-3 kg·m² lever is ~400 rad/s, which a 5 ms explicit step does
    not hold.

**The contact** is a penalty model on the foot's collision sphere (22 mm, `FOOT_RADIUS_M`): against the bar's capsule
-- a spring on the overlap along the line between their nearest points, damping on the closing speed, Coulomb
friction against sliding along or around the bar -- and against the door's face, so a foot that misses the bar does
not go through the door. The knee is checked against the door's face as well (`KNEE_RADIUS_M`). Everything is a force
on the foot in the world frame; the environment applies it at the foot sphere's centre.

**The task layer** (`HandleTaskLayer`) is what a robot would run, knowing the handle from an estimate -- the pivot,
the lever's direction and length, how far it turns -- and a rating of how hard it turns. From a pre-contact point
above the estimated bar it runs a reference down onto the bar and then round the bar's arc to the estimated end stop
plus an `overdrive`, holds it there for `hold_s`, and lifts back off. It reads only the foot's position, as the leg's
kinematics give it, and turns that into the lever angle it implies (`measured_theta`). The goal it hands the policy is
where the foot's centre should be at the reference. With `force_law` on (the force-aware policy) it also commands a
force along the arc's tangent: a PI law on the arc length the foot lags the reference by, the mechanism task's
(F-106), capped at `max_n` sized from the rating -- so the force it will put into an end stop is bounded by what it
was told the handle needs. Without it (the position-only policy) the goal is all there is.

**The key** (2026-10-21) is the same lever kinematics with three differences, per environment (`place(...,
pad=True)`): the lever is Lukas's lever key in the combiner box's cam lock (`leg_cfg`, "the combiner box's key"), so
its contact is the foot sphere against two boxes, the key's bar and its flat platform, instead of a capsule; it starts
on a stop at `theta_min` (-45 degrees, pointing up) instead of 0; and it resists with a lock's Coulomb friction that
*sticks* below its level, plus a cam detent before the unlock angle, and no return spring -- a regularised friction
would let the key creep round under any torque at all, and a cam lock stays where it is turned.

Plain torch, no Isaac imports (`tests/test_handle.py`). Every number is invented or taken from common lever-handle
hardware; none is measured.
"""
from __future__ import annotations

import math

import torch

from .insertion import FRICTION_V_EPS, unit

#: The Go2 foot's collision sphere (`description/go2_d1.urdf`, `FR_foot`): radius and its centre in the foot's frame.
FOOT_RADIUS_M = 0.022
FOOT_CENTRE_LOCAL_M = (-0.002, 0.0, 0.0)
#: The knee, as a sphere about the calf body's origin, for the door-face check only.
KNEE_RADIUS_M = 0.03
#: Penalty contact, capped: no spring force past `MAX_OVERLAP_M` of overlap, so a foot that starts somewhere unphysical
#: is not flung. 16,000 N/m (the key insertion's, stable against the arm) with a 12 mm cap until recipe j (2026-10-20
#: 10:55): on the foot it rang -- on and off the bar every step or two, 135-229 N peaks against step means under 63 N,
#: a contact mode near 28 Hz that a 50 Hz policy cannot hold. The Go2's foot is a rubber ball; 8,000 N/m and 20 mm.
CONTACT_K_N_PER_M = 8000.0
CONTACT_C_N_S_PER_M = 40.0
MAX_OVERLAP_M = 0.020
#: The end stops: a stiff one-sided spring and damper on the angle.
STOP_K_NM_PER_RAD = 200.0
STOP_C_NM_S_PER_RAD = 0.5
#: Below this spindle speed, rad/s, the Coulomb friction is ramped rather than switched (as `FRICTION_V_EPS` is).
SPINDLE_W_EPS = 0.2
#: Sub-steps of the lever's dynamics per physics step (see the module docstring).
SUBSTEPS = 8
#: A sticking lock (the key's): below this speed, rad/s, the key is held by its friction while the torque on it is
#: under the friction's level.
STICK_W = 1e-3

#: Task-layer phases.
IDLE, PRESSING, HOLDING, RELEASING, DONE = 0, 1, 2, 3, 4


def _rot_plane(u0: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """(z, a): world up, and the spindle axis `z x u0` positive theta turns the lever about."""
    z = torch.zeros_like(u0)
    z[:, 2] = 1.0
    return z, unit(torch.cross(z, u0, dim=-1))


def lever_dir(u0: torch.Tensor, theta: torch.Tensor) -> torch.Tensor:
    """The lever's direction at `theta`: `cos(theta) u0 - sin(theta) z`."""
    z, _ = _rot_plane(u0)
    return torch.cos(theta).unsqueeze(-1) * u0 - torch.sin(theta).unsqueeze(-1) * z


def lever_top(u0: torch.Tensor, theta: torch.Tensor) -> torch.Tensor:
    """The bar's upper side at `theta` (the side a foot pushes it down from): `sin(theta) u0 + cos(theta) z`."""
    z, _ = _rot_plane(u0)
    return torch.sin(theta).unsqueeze(-1) * u0 + torch.cos(theta).unsqueeze(-1) * z


def foot_on_bar(pivot: torch.Tensor, u0: torch.Tensor, theta: torch.Tensor, reach: torch.Tensor,
                clearance: torch.Tensor) -> torch.Tensor:
    """Where the foot sphere's centre sits resting on the bar's top at `reach` from the pivot, the lever at `theta`.

    `clearance` is the foot's radius plus the bar's. In the plane of the turn, with axes (u0, -z), the point is
    `reach (cos t, sin t) + clearance (sin t, -cos t)`: radius `hypot(reach, clearance)` at polar angle
    `t - atan2(clearance, reach)`, which is what `measured_theta` inverts.
    """
    return (pivot + reach.unsqueeze(-1) * lever_dir(u0, theta)
            + clearance.unsqueeze(-1) * lever_top(u0, theta))


def measured_theta(foot: torch.Tensor, pivot: torch.Tensor, u0: torch.Tensor, reach: torch.Tensor,
                   clearance: torch.Tensor) -> torch.Tensor:
    """The lever angle a foot resting on the bar at `foot` implies -- the inverse of `foot_on_bar`, using only the
    foot's position and the handle's (estimated) geometry. Off the bar it is still defined; it is just not an angle."""
    z, _ = _rot_plane(u0)
    r = foot - pivot
    polar = torch.atan2(-(r * z).sum(-1), (r * u0).sum(-1))
    return polar + torch.atan2(clearance, reach)


class LeverHandle:
    """A batch of lever handles on doors, one per environment; inactive until `place`d."""

    def __init__(self, num_envs: int, device, *, k: float = CONTACT_K_N_PER_M, c: float = CONTACT_C_N_S_PER_M) -> None:
        self.n_envs, self.device, self.k, self.c = num_envs, torch.device(device), k, c
        zeros = lambda *shape: torch.zeros(*shape, device=self.device)
        self.active = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.door, self.n, self.pivot, self.u0 = zeros(num_envs, 3), zeros(num_envs, 3), zeros(num_envs, 3), zeros(num_envs, 3)
        self.n[:, 0] = -1.0
        self.u0[:, 1] = 1.0
        for name in ("length", "inner", "radius", "inertia", "preload", "stiffness", "damping", "friction",
                     "theta_max", "theta_latch", "mu", "theta_min", "detent", "detent_from", "gravity_nm"):
            setattr(self, name, zeros(num_envs))
        #: The key's (`place(..., pad=True)`): box contact, a sticking friction, a start stop at `theta_min`.
        self.pad = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.inertia.fill_(1e-3)
        self.theta, self.omega = zeros(num_envs), zeros(num_envs)
        #: Outputs of the last `step`, averaged over its sub-steps: the force on the foot (world), the bar's and the
        #: door's normal forces, and the force's component along the bar's turning direction (what turns it).
        self.force_on_foot = zeros(num_envs, 3)
        self.bar_n, self.door_n, self.knee_door_n = zeros(num_envs), zeros(num_envs), zeros(num_envs)
        self.drive_n = zeros(num_envs)
        self.knee_force, self.head_force = zeros(num_envs, 3), zeros(num_envs, 3)
        #: Largest total contact force seen at any sub-step since the last `take_peak` (the plan's peak force).
        self._peak = zeros(num_envs)
        self.on_bar = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.touched = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        #: Where along the bar the foot last touched it (m from the pivot), for the slip-off check.
        self.contact_reach = zeros(num_envs)

    # --- configuration ------------------------------------------------------------------------------------------

    def place(self, ids: torch.Tensor, *, door: torch.Tensor, normal: torch.Tensor, u0: torch.Tensor,
              standoff: torch.Tensor, length: torch.Tensor, inner: torch.Tensor, radius: torch.Tensor,
              inertia: torch.Tensor, rest_nm: torch.Tensor, full_nm: torch.Tensor, damping: torch.Tensor,
              friction: torch.Tensor, theta_max: torch.Tensor, theta_latch: torch.Tensor, mu: torch.Tensor,
              theta_min: torch.Tensor | None = None, pad: bool = False, detent_nm: torch.Tensor | None = None,
              detent_from: torch.Tensor | None = None, gravity_nm: torch.Tensor | None = None) -> None:
        """Mount a handle for `ids`: `rest_nm` is the spring's torque at rest (its preload), `full_nm` at the stop.

        A key (`pad=True`): `theta_min` its start stop, `friction` its lock's sticking friction, `detent_nm` more of it
        from `detent_from` to `theta_latch` (the cam), `gravity_nm` its weight's torque about the shaft when level;
        `length`, `inner`, `radius` and `standoff` are unused (its shape is `leg_cfg.KEY_*_BOX`)."""
        self.door[ids], self.n[ids], self.u0[ids] = door, unit(normal), unit(u0)
        self.pivot[ids] = door + standoff.unsqueeze(-1) * self.n[ids]
        self.length[ids], self.inner[ids], self.radius[ids], self.inertia[ids] = length, inner, radius, inertia
        self.preload[ids] = rest_nm
        self.stiffness[ids] = (full_nm - rest_nm).clamp(min=0.0) / theta_max.clamp(min=1e-3)
        self.damping[ids], self.friction[ids] = damping, friction
        self.theta_max[ids], self.theta_latch[ids], self.mu[ids] = theta_max, theta_latch, mu
        zero = torch.zeros_like(theta_max)
        self.theta_min[ids] = zero if theta_min is None else theta_min
        self.detent[ids] = zero if detent_nm is None else detent_nm
        self.detent_from[ids] = theta_latch if detent_from is None else detent_from
        self.gravity_nm[ids] = zero if gravity_nm is None else gravity_nm
        self.pad[ids] = pad
        self.theta[ids], self.omega[ids] = self.theta_min[ids], 0.0
        self.touched[ids] = False
        self.active[ids] = True

    def reset(self, ids: torch.Tensor) -> None:
        self.active[ids] = False
        self.theta[ids], self.omega[ids] = 0.0, 0.0
        for buf in (self.force_on_foot, self.bar_n, self.door_n, self.knee_door_n, self.drive_n, self._peak,
                    self.contact_reach):
            buf[ids] = 0.0
        self.on_bar[ids], self.touched[ids] = False, False

    def spring_nm(self, theta: torch.Tensor | None = None) -> torch.Tensor:
        """The return spring's torque at `theta` (default: now), N·m."""
        theta = self.theta if theta is None else theta
        return self.preload + self.stiffness * (theta - self.theta_min).clamp(min=0.0)

    def latch_open(self) -> torch.Tensor:
        return self.active & (self.theta >= self.theta_latch)

    def take_peak(self) -> torch.Tensor:
        peak = self._peak.clone()
        self._peak.zero_()
        return peak

    # --- the contact ------------------------------------------------------------------------------------------------

    def _bar_contact(self, foot: torch.Tensor, foot_v: torch.Tensor, foot_r: float):
        """Force on the foot from the bar (world), the contact point, and whether they touch."""
        u = lever_dir(self.u0, self.theta)
        _, axis = _rot_plane(self.u0)
        start = self.pivot + self.inner.unsqueeze(-1) * u
        span = (self.length - self.inner).clamp(min=1e-4)
        along = (((foot - start) * u).sum(-1) / span).clamp(0.0, 1.0)
        nearest = start + (along * span).unsqueeze(-1) * u
        gap = foot - nearest
        dist = gap.norm(dim=-1).clamp(min=1e-6)
        normal = gap / dist.unsqueeze(-1)
        overlap = (foot_r + self.radius - dist).clamp(min=0.0)
        touching = self.active & (overlap > 0.0)
        surface_v = self.omega.unsqueeze(-1) * torch.cross(axis, nearest - self.pivot, dim=-1)
        rel = foot_v - surface_v
        closing = (rel * normal).sum(-1)
        # The damping is bounded by the spring (Hunt-Crossley's point: no force without overlap). Unbounded, a lever
        # snapping back on its spring after the foot slipped struck it at metres a second and the damping alone made
        # 400 N spikes inside one policy step (the stage-1 zero-shot evaluation, 2026-10-20) -- "damage" from the
        # contact model, not from the policy.
        spring = self.k * overlap.clamp(max=MAX_OVERLAP_M)
        f_n = (spring + (-self.c * closing).clamp(min=-spring, max=spring)).clamp(min=0.0) * touching
        slide = rel - closing.unsqueeze(-1) * normal
        speed = slide.norm(dim=-1)
        friction = (-(self.mu * f_n).unsqueeze(-1) * slide / speed.clamp(min=1e-9).unsqueeze(-1)
                    * torch.tanh(speed / FRICTION_V_EPS).unsqueeze(-1))
        return f_n.unsqueeze(-1) * normal + friction, nearest, touching, f_n, along * span + self.inner

    def _pad_contact(self, foot: torch.Tensor, foot_v: torch.Tensor, foot_r: float):
        """The key's: force on the foot from the nearer of the key's two boxes (its bar and its platform), in the key's
        frame (along it, off the door, up its top side) about the shaft's point on the door's face."""
        from . import leg_cfg

        u, top, n = lever_dir(self.u0, self.theta), lever_top(self.u0, self.theta), self.n
        _, axis = _rot_plane(self.u0)
        rel = foot - self.door
        local = torch.stack(((rel * u).sum(-1), (rel * n).sum(-1), (rel * top).sum(-1)), dim=-1)
        eye = torch.eye(3, device=foot.device, dtype=foot.dtype)
        faces = torch.cat((-eye, eye), dim=0)                          # -e0 -e1 -e2 +e0 +e1 +e2
        best_overlap = torch.full_like(local[:, 0], -1e9)
        best_normal, best_point = torch.zeros_like(local), torch.zeros_like(local)
        for box in (leg_cfg.KEY_BAR_BOX, leg_cfg.KEY_PAD_BOX):
            lo = torch.tensor([b[0] for b in box], device=foot.device, dtype=foot.dtype)
            hi = torch.tensor([b[1] for b in box], device=foot.device, dtype=foot.dtype)
            q = torch.maximum(torch.minimum(local, hi), lo)
            gap = local - q
            dist = gap.norm(dim=-1)
            inside = dist < 1e-9
            depth, face = torch.cat((local - lo, hi - local), dim=-1).min(dim=-1)
            n_in = faces[face]
            normal = torch.where(inside.unsqueeze(-1), n_in, gap / dist.clamp(min=1e-9).unsqueeze(-1))
            overlap = torch.where(inside, foot_r + depth, foot_r - dist)
            point = torch.where(inside.unsqueeze(-1), local + depth.unsqueeze(-1) * n_in, q)
            better = overlap > best_overlap
            best_overlap = torch.where(better, overlap, best_overlap)
            best_normal = torch.where(better.unsqueeze(-1), normal, best_normal)
            best_point = torch.where(better.unsqueeze(-1), point, best_point)
        to_world = lambda v: v[:, 0:1] * u + v[:, 1:2] * n + v[:, 2:3] * top
        normal_w = to_world(best_normal)
        point_w = self.door + to_world(best_point)
        overlap = best_overlap.clamp(min=0.0)
        touching = self.active & (overlap > 0.0)
        surface_v = self.omega.unsqueeze(-1) * torch.cross(axis, point_w - self.door, dim=-1)
        rel_v = foot_v - surface_v
        closing = (rel_v * normal_w).sum(-1)
        spring = self.k * overlap.clamp(max=MAX_OVERLAP_M)
        f_n = (spring + (-self.c * closing).clamp(min=-spring, max=spring)).clamp(min=0.0) * touching
        slide = rel_v - closing.unsqueeze(-1) * normal_w
        speed = slide.norm(dim=-1)
        friction = (-(self.mu * f_n).unsqueeze(-1) * slide / speed.clamp(min=1e-9).unsqueeze(-1)
                    * torch.tanh(speed / FRICTION_V_EPS).unsqueeze(-1))
        return f_n.unsqueeze(-1) * normal_w + friction, point_w, touching, f_n, best_point[:, 0]

    def _plane_contact(self, point: torch.Tensor, vel: torch.Tensor, radius: float, mu: torch.Tensor):
        """Force on a sphere from the door's face (world) and its normal magnitude."""
        height = ((point - self.door) * self.n).sum(-1)
        overlap = (radius - height).clamp(min=0.0)
        touching = self.active & (overlap > 0.0)
        closing = -(vel * self.n).sum(-1)
        spring = self.k * overlap.clamp(max=MAX_OVERLAP_M)
        f_n = (spring + (self.c * closing).clamp(min=-spring, max=spring)).clamp(min=0.0) * touching
        slide = vel - (vel * self.n).sum(-1, keepdim=True) * self.n
        speed = slide.norm(dim=-1)
        friction = (-(mu * f_n).unsqueeze(-1) * slide / speed.clamp(min=1e-9).unsqueeze(-1)
                    * torch.tanh(speed / FRICTION_V_EPS).unsqueeze(-1))
        return f_n.unsqueeze(-1) * self.n + friction, f_n

    def step(self, foot: torch.Tensor, foot_v: torch.Tensor, dt: float, *, foot_r: float = FOOT_RADIUS_M,
             knee: torch.Tensor | None = None, knee_v: torch.Tensor | None = None,
             head: torch.Tensor | None = None, head_v: torch.Tensor | None = None,
             head_r: float = 0.065) -> torch.Tensor:
        """Advance one physics step of `dt` with the foot at `foot` moving at `foot_v` (world); return the mean force
        on the foot over the step. The knee, if given, is checked against the door's face (its force is returned in
        `knee_force`, for the environment to apply to the calf)."""
        h = dt / SUBSTEPS
        _, axis = _rot_plane(self.u0)
        total = torch.zeros_like(foot)
        bar_sum, door_sum, drive_sum = (torch.zeros_like(self.theta) for _ in range(3))
        any_touch = torch.zeros_like(self.active)
        reach_last = self.contact_reach.clone()
        keys = bool(self.pad.any())
        for k in range(SUBSTEPS):
            here = foot + foot_v * ((k + 0.5) * h)
            f_bar, point, touching, f_n, reach = self._bar_contact(here, foot_v, foot_r)
            if keys:
                pad = self._pad_contact(here, foot_v, foot_r)
                m = self.pad
                f_bar, point = torch.where(m.unsqueeze(-1), pad[0], f_bar), torch.where(m.unsqueeze(-1), pad[1], point)
                touching, f_n, reach = torch.where(m, pad[2], touching), torch.where(m, pad[3], f_n), torch.where(m, pad[4], reach)
            f_door, door_n = self._plane_contact(here, foot_v, foot_r, self.mu)
            on_foot = f_bar + f_door
            # The bar takes the opposite of what it puts on the foot, at the contact point.
            torque = (torch.cross(point - self.pivot, -f_bar, dim=-1) * axis).sum(-1)
            tangent = torch.cross(axis, unit(point - self.pivot), dim=-1)
            drive_sum += (-f_bar * tangent).sum(-1)
            resist = self.spring_nm() + self.damping * self.omega \
                + self.friction * torch.tanh(self.omega / SPINDLE_W_EPS)
            below = (self.theta_min - self.theta).clamp(min=0.0)
            above = (self.theta - self.theta_max).clamp(min=0.0)
            stops = (STOP_K_NM_PER_RAD * below - STOP_C_NM_S_PER_RAD * self.omega * (below > 0)
                     - STOP_K_NM_PER_RAD * above - STOP_C_NM_S_PER_RAD * self.omega * (above > 0))
            # The spring pushes the lever back up only while it is off its rest stop; at the stop the stop carries it.
            resist = torch.where(self.theta > self.theta_min, resist, torch.minimum(resist, torque.clamp(min=0.0)))
            accel = (torque - resist + stops) / self.inertia
            omega = self.omega + h * accel
            if keys:
                omega = torch.where(self.pad, self._key_omega(torque, stops, h), omega)
            self.omega = torch.where(self.active, omega, torch.zeros_like(self.omega))
            self.theta = torch.where(self.active, self.theta + h * self.omega, torch.zeros_like(self.theta))
            total += on_foot
            bar_sum += f_n
            door_sum += door_n
            any_touch |= touching
            reach_last = torch.where(touching, reach, reach_last)
            self._peak = torch.maximum(self._peak, on_foot.norm(dim=-1))
        self.force_on_foot = total / SUBSTEPS
        self.bar_n, self.door_n, self.drive_n = bar_sum / SUBSTEPS, door_sum / SUBSTEPS, drive_sum / SUBSTEPS
        self.on_bar = any_touch
        self.touched |= any_touch
        self.contact_reach = reach_last
        self.knee_force = torch.zeros_like(foot)
        if knee is not None:
            self.knee_force, self.knee_door_n = self._plane_contact(knee, knee_v, KNEE_RADIUS_M, self.mu)
            self._peak = torch.maximum(self._peak, self.knee_force.norm(dim=-1))
        # The robot's head, if given, against the door's face (the key's task stands close to it): its force is
        # `head_force`, counted with the knee's in `knee_door_n` (the door-push penalty), not in the peak.
        self.head_force = torch.zeros_like(foot)
        if head is not None:
            self.head_force, head_n = self._plane_contact(head, head_v, head_r, self.mu)
            self.knee_door_n = self.knee_door_n + head_n
        return self.force_on_foot


    def _key_omega(self, torque: torch.Tensor, stops: torch.Tensor, h: float) -> torch.Tensor:
        """A key's next speed: its lock's Coulomb friction (plus the cam's detent in its window) holds it while the rest
        of the torque on it is under that level, and opposes it while it turns; a turn that would reverse stops."""
        drive = torque + self.gravity_nm * torch.cos(self.theta) - self.damping * self.omega + stops
        cam = (self.theta >= self.detent_from) & (self.theta < self.theta_latch)
        level = self.friction + self.detent * cam
        stuck = self.omega.abs() < STICK_W
        breaks = drive.abs() > level
        start = torch.where(breaks, (drive - level * torch.sign(drive)) / self.inertia * h, torch.zeros_like(drive))
        moving = self.omega + h * (drive - level * torch.sign(self.omega)) / self.inertia
        moving = torch.where(torch.sign(moving) != torch.sign(self.omega), torch.zeros_like(moving), moving)
        return torch.where(stuck, start, moving)


class HandleTaskLayer:
    """The turn as a robot would drive it from an estimate of the handle: a reference onto the bar and round its arc,
    and -- for a force-aware policy -- a PI force along the arc on the foot's lag. Only the foot's position is read."""

    def __init__(self, num_envs: int, device, *, kp: float, ki: float) -> None:
        self.n_envs, self.device, self.kp, self.ki = num_envs, torch.device(device), kp, ki
        zeros = lambda *shape: torch.zeros(*shape, device=self.device)
        self.phase = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.pivot, self.u0 = zeros(num_envs, 3), zeros(num_envs, 3)
        self.u0[:, 1] = 1.0
        for name in ("reach", "clearance", "theta_end", "descent", "descend_speed", "turn_speed", "max_n",
                     "touch_n", "hold_s", "timer", "ref", "integral", "release_s", "touch_s", "touch_timer",
                     "theta_start"):
            setattr(self, name, zeros(num_envs))
        self.force_law = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.goal = zeros(num_envs, 3)
        self.force_cmd = zeros(num_envs, 3)
        self.tangent = zeros(num_envs, 3)
        self.tangent[:, 2] = -1.0
        #: The reference as a lever angle (rad; negative while still descending onto the bar) and the measured one.
        self.theta_ref, self.theta_meas = zeros(num_envs), zeros(num_envs)

    def start(self, ids: torch.Tensor, *, pivot: torch.Tensor, u0: torch.Tensor, reach: torch.Tensor,
              clearance: torch.Tensor, theta_end: torch.Tensor, descent: torch.Tensor, descend_speed: torch.Tensor,
              turn_speed: torch.Tensor, max_n: torch.Tensor, touch_n: torch.Tensor, hold_s: torch.Tensor,
              release_s: torch.Tensor, force_law: torch.Tensor, touch_s: torch.Tensor | None = None,
              theta_start: torch.Tensor | None = None) -> None:
        """Begin pressing for `ids`, from the pre-contact point `descent` above the estimated bar. `touch_s`: how long
        the reference waits at the bar's top before turning it, the force law capped at `touch_n` meanwhile.
        `theta_start`: the angle the lever starts at (0, level, for a handle; the key's -45 degrees), the descent
        coming onto its top along its normal there."""
        self.theta_start[ids] = torch.zeros_like(reach) if theta_start is None else theta_start
        self.pivot[ids], self.u0[ids] = pivot, unit(u0)
        self.reach[ids], self.clearance[ids], self.theta_end[ids] = reach, clearance, theta_end
        self.descent[ids], self.descend_speed[ids], self.turn_speed[ids] = descent, descend_speed, turn_speed
        self.max_n[ids], self.touch_n[ids], self.hold_s[ids], self.release_s[ids] = max_n, touch_n, hold_s, release_s
        self.force_law[ids] = force_law
        self.phase[ids] = PRESSING
        self.timer[ids], self.ref[ids], self.integral[ids] = 0.0, 0.0, 0.0
        self.touch_s[ids] = 0.0 if touch_s is None else touch_s
        self.touch_timer[ids] = 0.0
        self.force_cmd[ids] = 0.0
        self.goal[ids] = self.pre_contact()[ids]

    def reset(self, ids: torch.Tensor) -> None:
        self.phase[ids] = IDLE
        for buf in (self.timer, self.ref, self.integral, self.theta_ref, self.theta_meas, self.touch_timer):
            buf[ids] = 0.0
        self.force_cmd[ids] = 0.0
        self.goal[ids] = 0.0

    def pre_contact(self) -> torch.Tensor:
        """The foot's centre `descent` off where it would rest on the estimated bar at its start, along the bar's top
        side's normal (straight up for a level handle)."""
        rest = foot_on_bar(self.pivot, self.u0, self.theta_start, self.reach, self.clearance)
        return rest + self.descent.unsqueeze(-1) * lever_top(self.u0, self.theta_start)

    def path(self, s: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """The reference at arc length `s` from the pre-contact point: the foot's centre, the path's unit tangent,
        and the lever angle it stands for (negative while descending)."""
        normal = lever_top(self.u0, self.theta_start)
        down = s.clamp(max=1e9) < self.descent
        theta = self.theta_start + ((s - self.descent) / self.reach.clamp(min=1e-3)).clamp(min=0.0)
        on_bar = foot_on_bar(self.pivot, self.u0, theta, self.reach, self.clearance)
        above = foot_on_bar(self.pivot, self.u0, self.theta_start, self.reach, self.clearance) \
            + (self.descent - s).clamp(min=0.0).unsqueeze(-1) * normal
        point = torch.where(down.unsqueeze(-1), above, on_bar)
        tangent = torch.where(down.unsqueeze(-1), -normal, -lever_top(self.u0, theta))
        angle = torch.where(down, self.theta_start - (self.descent - s) / self.reach.clamp(min=1e-3), theta)
        return point, tangent, angle

    def step(self, foot: torch.Tensor, dt: float) -> tuple[torch.Tensor, torch.Tensor]:
        """Advance one policy step; return (the goal for the foot's centre, the force command), both world."""
        active = self.phase != IDLE
        self.theta_meas = measured_theta(foot, self.pivot, self.u0, self.reach, self.clearance)
        end = self.descent + self.reach * (self.theta_end - self.theta_start)
        speed = torch.where(self.ref < self.descent, self.descend_speed, self.turn_speed * self.reach)
        pressing = self.phase == PRESSING
        # Touch, then turn: the reference waits at the bar's top for `touch_s` (2026-10-20: pushing as it arrived, the
        # foot was carried 3-4 cm toward the robot and met the bar on its front shoulder).
        candidate = torch.minimum(self.ref + speed * dt, end)
        pausing = pressing & (candidate >= self.descent) & (self.touch_timer < self.touch_s)
        self.touch_timer = torch.where(pausing, self.touch_timer + dt, self.touch_timer)
        self.ref = torch.where(pressing, torch.where(pausing, torch.minimum(candidate, self.descent), candidate), self.ref)
        arrived = pressing & (self.ref >= end)
        self.phase = torch.where(arrived, torch.full_like(self.phase, HOLDING), self.phase)
        self.timer = torch.where(arrived, torch.zeros_like(self.timer), self.timer)
        holding = self.phase == HOLDING
        self.timer = torch.where(holding | (self.phase == RELEASING), self.timer + dt, self.timer)
        done_holding = holding & (self.timer >= self.hold_s)
        self.phase = torch.where(done_holding, torch.full_like(self.phase, RELEASING), self.phase)
        self.timer = torch.where(done_holding, torch.zeros_like(self.timer), self.timer)
        releasing = self.phase == RELEASING
        # The release runs the reference back up the path to the pre-contact point.
        back = (self.ref - end * dt / self.release_s.clamp(min=dt)).clamp(min=0.0)
        self.ref = torch.where(releasing, back, self.ref)
        finished = releasing & (self.ref <= 0.0)
        self.phase = torch.where(finished, torch.full_like(self.phase, DONE), self.phase)

        point, tangent, angle = self.path(self.ref)
        self.theta_ref = angle
        #: The path's direction at the reference: the way the foot pushes (for the outer loop to leave alone).
        self.tangent = tangent
        # The force law: arc length the foot lags the reference by, along the turn, once the reference is on the bar.
        on_arc = (pressing | holding) & (self.ref >= self.descent)
        lag = (self.reach * (self.theta_ref - self.theta_meas)).clamp(min=-0.02, max=0.05)
        self.integral = torch.where(on_arc & self.force_law, (self.integral + lag * dt).clamp(min=0.0), self.integral)
        self.integral = torch.where(on_arc, self.integral, torch.zeros_like(self.integral))
        law = (self.touch_n + self.kp * lag + self.ki * self.integral).clamp(min=0.0)
        law = torch.minimum(law, torch.where(pausing, self.touch_n, self.max_n))
        magnitude = torch.where(on_arc & self.force_law, law, torch.zeros_like(law))
        self.force_cmd = magnitude.unsqueeze(-1) * tangent
        self.goal = torch.where(active.unsqueeze(-1), point, self.goal)
        return self.goal, self.force_cmd

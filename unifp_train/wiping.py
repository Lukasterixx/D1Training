"""Wiping a flat surface with a pad held in the jaws: the plant, the path, and the task layer that drives the wipe.

The plan's wiping task (`docs/thesis_b_plan.md`, "Contact-task progression"): a planar fixture with a known normal, a
slow prescribed path, a rigid pad initially; success is at least 90% of the path wiped and contact during at least 80%
of the motion. Built as the key insertion is (`insertion.py`): an analytic plant stepped at the physics rate, and a
task layer that drives the wipe from an estimate of where the surface is.

**The pad** is a rigid flat disc of radius `PAD_RADIUS_M`, its face normal the hand's approach, its centre
`PAD_LENGTH_M` beyond the jaw centre: a sponge block on a short holder. It meets the surface at `PAD_POINTS` points --
its centre and a ring of eight on its rim -- each a share of one penalty spring, so a pad held flat presses with its
whole face and a tilted one rides on its rim, with the contact moved there. The force acts at the centre of pressure.

**The surface** is a plane through `origin` with outward normal `normal` (toward the robot), a panel of half sizes
`half_a` by `half_b` along its in-plane axes `a` (horizontal) and `b`. Each pad point past it is pushed back out along
the normal, damped while it goes further in, and feels Coulomb friction against its own sliding, ramped below
`insertion.FRICTION_V_EPS`. Past the panel's edge nothing is touched; a point `FACE_DEPTH_M` into it has gone through.

**The path** is a polyline in the plane's (a, b) coordinates, resampled to `PATH_SAMPLES` points evenly spaced along
it: a straight stroke (`line_path`) or strokes joined into a zigzag (`zigzag_path`).

**The task layer** (`WipeTaskLayer`) is what a robot would run, knowing the surface only from an estimate: the pad to a
standoff off the estimated surface over the path's start; then a PI force law along the estimated normal on how far
the pad lags a reference depth that runs to `overdrive` past the estimated surface, capped at the wiping force -- so
wherever the true surface is within the overdrive, the pad on it pushes with exactly that force; once the pad has
stalled against the surface, a reference point along the path at the wiping speed, never more than `lead_m` ahead of
the pad's measured progress, with a PI force across the normal on the pad's miss of it (friction has to be pushed
through, and UniFP's controller yields to what it measures); at the path's end, lift off. Only the pad's position is
read.

Plain torch, no Isaac imports (`tests/test_wiping.py`). Every number is invented; none is measured.
"""
from __future__ import annotations

import math

import torch

from .insertion import FRICTION_V_EPS, unit

#: The pad: a 50 mm disc whose face is 45 mm beyond the jaw centre (a 25 mm holder past the 20 mm fingertips).
PAD_RADIUS_M = 0.025
PAD_LENGTH_M = 0.045
#: Penalty contact over the whole pad: the key insertion's stiffness and damping, split over the pad's points.
CONTACT_K_N_PER_M = 16000.0
CONTACT_C_N_S_PER_M = 50.0
#: The pad's contact points: its centre and a ring of this many on its rim.
RIM_POINTS = 8
PAD_POINTS = RIM_POINTS + 1
#: A normal force at least this is contact, N; below it the pad is resting on the surface's noise.
CONTACT_MIN_N = 1.0
#: A pad point this far into the panel has gone through it (damage); no penalty force grows past `MAX_OVER_M`.
FACE_DEPTH_M = 0.01
MAX_OVER_M = 0.01
#: The path, resampled to this many points evenly spaced along it.
PATH_SAMPLES = 64
#: A path sample is wiped when the pad's centre of pressure passes within this of it, in contact: a little under the
#: pad's radius, so a pad dragged along the path's side does not count.
WIPE_RADIUS_M = 0.02


# --- the path ---------------------------------------------------------------------------------------------------


def resample(vertices: torch.Tensor, samples: int = PATH_SAMPLES) -> tuple[torch.Tensor, torch.Tensor]:
    """(points (count, samples, 2), arc length at each (count, samples)): the polyline `vertices` (count, K, 2)
    resampled at even spacing along it."""
    seg = (vertices[:, 1:] - vertices[:, :-1]).norm(dim=-1)
    cum = torch.cat((torch.zeros_like(seg[:, :1]), torch.cumsum(seg, dim=-1)), dim=-1)      # (count, K)
    total = cum[:, -1:]
    arc = total * torch.linspace(0.0, 1.0, samples, device=vertices.device).unsqueeze(0)
    index = torch.searchsorted(cum.contiguous(), arc.contiguous(), right=True).clamp(1, vertices.shape[1] - 1)
    lo, hi = cum.gather(1, index - 1), cum.gather(1, index)
    t = ((arc - lo) / (hi - lo).clamp(min=1e-9)).clamp(0.0, 1.0).unsqueeze(-1)
    a = vertices.gather(1, (index - 1).unsqueeze(-1).expand(-1, -1, 2))
    b = vertices.gather(1, index.unsqueeze(-1).expand(-1, -1, 2))
    return a + t * (b - a), arc


def line_path(length: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    """A straight stroke of `length` through the origin at `angle` from the in-plane horizontal: vertices (count, 2, 2).
    A negative length runs it the other way."""
    direction = torch.stack((torch.cos(angle), torch.sin(angle)), dim=-1)
    half = (0.5 * length).unsqueeze(-1) * direction
    return torch.stack((-half, half), dim=1)


def zigzag_path(length: torch.Tensor, spacing: torch.Tensor, strokes: int = 3) -> torch.Tensor:
    """`strokes` horizontal strokes of `length`, `spacing` apart top to bottom, each joined to the next at alternate
    ends, centred on the origin: vertices (count, 2 * strokes, 2). A negative length starts from the other side."""
    rows = []
    for i in range(strokes):
        b = (0.5 * (strokes - 1) - i) * spacing
        side = 0.5 * length * (1.0 if i % 2 == 0 else -1.0)
        rows.append(torch.stack((-side, b), dim=-1))
        rows.append(torch.stack((side, b), dim=-1))
    return torch.stack(rows, dim=1)


def point_at(points: torch.Tensor, arc: torch.Tensor, s: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """(the path's point at arc length `s`, its unit tangent there), each (count, 2)."""
    samples = points.shape[1]
    index = torch.searchsorted(arc.contiguous(), s.unsqueeze(-1).contiguous(), right=True).clamp(1, samples - 1)
    lo, hi = arc.gather(1, index - 1).squeeze(-1), arc.gather(1, index).squeeze(-1)
    t = ((s - lo) / (hi - lo).clamp(min=1e-9)).clamp(0.0, 1.0).unsqueeze(-1)
    a = points.gather(1, (index - 1).unsqueeze(-1).expand(-1, -1, 2)).squeeze(1)
    b = points.gather(1, index.unsqueeze(-1).expand(-1, -1, 2)).squeeze(1)
    tangent = b - a
    return a + t * tangent, tangent / tangent.norm(dim=-1, keepdim=True).clamp(min=1e-9)


def wiped_samples(points: torch.Tensor, contact_ab: torch.Tensor, in_contact: torch.Tensor,
                  radius: float = WIPE_RADIUS_M) -> torch.Tensor:
    """(N, PATH_SAMPLES) bool: the path samples the contact at `contact_ab` (N, 2) wipes now, where `in_contact`."""
    near = (points - contact_ab.unsqueeze(1)).norm(dim=-1) < radius
    return near & in_contact.unsqueeze(-1)


def plane_axes(normal: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """(a, b): the plane's horizontal in-plane axis (`up x normal`) and `normal x a`, which points up the plane."""
    up = torch.zeros_like(normal)
    up[:, 2] = 1.0
    a = unit(torch.cross(up, normal, dim=-1))
    return a, torch.cross(normal, a, dim=-1)


# --- the surface --------------------------------------------------------------------------------------------------


def pad_points(centre: torch.Tensor, face: torch.Tensor, radius: float = PAD_RADIUS_M) -> torch.Tensor:
    """The pad's contact points, (N, PAD_POINTS, 3): its centre and a ring on its rim, in the plane normal to `face`."""
    f = unit(face)
    helper = torch.zeros_like(f)
    helper[:, 0] = 1.0
    helper = torch.where((f[:, 0].abs() > 0.9).unsqueeze(-1), torch.tensor([0.0, 1.0, 0.0], device=f.device), helper)
    e1 = unit(torch.cross(f, helper, dim=-1))
    e2 = torch.cross(f, e1, dim=-1)
    angles = torch.arange(RIM_POINTS, device=f.device, dtype=f.dtype) * (2 * math.pi / RIM_POINTS)
    ring = (torch.cos(angles)[None, :, None] * e1[:, None] + torch.sin(angles)[None, :, None] * e2[:, None]) * radius
    return torch.cat((centre[:, None], centre[:, None] + ring), dim=1)


class Surface:
    """One flat panel per environment, and the pad's contact with it."""

    def __init__(self, num_envs: int, device, *, k: float = CONTACT_K_N_PER_M, c: float = CONTACT_C_N_S_PER_M) -> None:
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.k, self.c = k / PAD_POINTS, c / PAD_POINTS
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        zeros3 = lambda: torch.zeros(num_envs, 3, device=self.device)
        flags = lambda: torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.origin, self.normal, self.a, self.b = zeros3(), zeros3(), zeros3(), zeros3()
        self.half_a, self.half_b, self.mu = zeros(), zeros(), zeros()
        self.active = flags()
        #: As of the last `step()`: the force on the pad (world) and where it acts, the normal force, the friction's
        #: size, the deepest pad point's depth past the surface, whether the pad is in contact, and gone through.
        self.force_on_pad = zeros3()
        self.pressure_centre = zeros3()
        self.normal_n = zeros()
        self.tangential_n = zeros()
        self.depth = zeros()
        self.in_contact = flags()
        self.pierced = flags()
        #: The largest normal force since `take_peak()`: the physics-rate peak.
        self.peak_n = zeros()

    def place(self, env_ids: torch.Tensor, *, origin: torch.Tensor, normal: torch.Tensor, half_a: torch.Tensor,
              half_b: torch.Tensor, mu: torch.Tensor) -> None:
        n = unit(normal)
        a, b = plane_axes(n)
        self.origin[env_ids], self.normal[env_ids], self.a[env_ids], self.b[env_ids] = origin, n, a, b
        self.half_a[env_ids], self.half_b[env_ids], self.mu[env_ids] = half_a, half_b, mu
        self.active[env_ids] = True
        self._clear(env_ids)

    def reset(self, env_ids: torch.Tensor) -> None:
        self.active[env_ids] = False
        self._clear(env_ids)

    def _clear(self, env_ids: torch.Tensor) -> None:
        self.force_on_pad[env_ids] = 0.0
        self.pressure_centre[env_ids] = 0.0
        for buf in (self.normal_n, self.tangential_n, self.depth, self.peak_n):
            buf[env_ids] = 0.0
        self.in_contact[env_ids] = False
        self.pierced[env_ids] = False

    def take_peak(self) -> torch.Tensor:
        peak = self.peak_n.clone()
        self.peak_n.zero_()
        return peak

    def step(self, centre: torch.Tensor, face: torch.Tensor, vel: torch.Tensor, omega: torch.Tensor) -> torch.Tensor:
        """The force on the pad (world, (N, 3)) with its centre at `centre`, its face along `face`, the centre moving at
        `vel` and the pad turning at `omega`. It acts at `pressure_centre`."""
        active = self.active
        pts = pad_points(centre, face)                                           # (N, P, 3)
        rel = pts - self.origin[:, None]
        depth = -(rel * self.normal[:, None]).sum(-1)                            # into the surface, positive
        on_panel = ((rel * self.a[:, None]).sum(-1).abs() < self.half_a[:, None]) \
            & ((rel * self.b[:, None]).sum(-1).abs() < self.half_b[:, None]) & active[:, None]
        into = depth.clamp(min=0.0) * on_panel
        pierced = self.pierced | (into >= FACE_DEPTH_M).any(dim=-1)
        over = into.clamp(max=MAX_OVER_M)
        touching = over > 0.0
        v = vel[:, None] + torch.cross(omega[:, None].expand_as(pts), pts - centre[:, None], dim=-1)
        v_in = -(v * self.normal[:, None]).sum(-1)                               # into the surface, positive
        normal = (self.k * over + self.c * v_in.clamp(min=0.0) * touching)       # (N, P)
        v_t = v + v_in.unsqueeze(-1) * self.normal[:, None]
        speed = v_t.norm(dim=-1, keepdim=True)
        friction = -(self.mu[:, None] * normal).unsqueeze(-1) * v_t / speed.clamp(min=1e-9) \
            * torch.tanh(speed / FRICTION_V_EPS)
        force_pts = normal.unsqueeze(-1) * self.normal[:, None] + friction
        total_n = normal.sum(-1)
        force = force_pts.sum(1)
        weight = normal / total_n.clamp(min=1e-9).unsqueeze(-1)
        pressure = torch.where((total_n > 0.0).unsqueeze(-1), (weight.unsqueeze(-1) * pts).sum(1), centre)

        self.force_on_pad = torch.where(active.unsqueeze(-1), force, torch.zeros_like(force))
        self.pressure_centre = pressure
        self.normal_n = total_n * active
        self.tangential_n = friction.sum(1).norm(dim=-1) * active
        self.depth = torch.where(active, depth.max(dim=-1).values, torch.zeros_like(total_n))
        self.in_contact = active & (self.normal_n >= CONTACT_MIN_N)
        self.pierced = pierced
        self.peak_n = torch.maximum(self.peak_n, self.normal_n)
        return self.force_on_pad

    def in_plane(self, point: torch.Tensor) -> torch.Tensor:
        """`point`'s (a, b) coordinates in this surface's plane, (N, 2)."""
        rel = point - self.origin
        return torch.stack(((rel * self.a).sum(-1), (rel * self.b).sum(-1)), dim=-1)


# --- the task layer -----------------------------------------------------------------------------------------------

#: `WipeTaskLayer.phase`.
APPROACHING, WIPING, LIFTING, DONE = 0, 1, 2, 3


class WipeTaskLayer:
    """The wipe as a robot would run it, from an estimate of the surface and the pad's measured position.

    Per environment: the estimated surface (origin, outward normal, in-plane axes), the path in its plane, the wiping
    force and speed. The goal it returns is for the pad's centre -- along the normal where the pad is while it pushes
    (the law carries the push), back out while it lifts; in the plane on the path's reference point -- and the force
    is the normal law's push into the estimated surface plus the in-plane law's push along the path.
    """

    def __init__(self, num_envs: int, device, *, kp: float = 400.0, ki: float = 1000.0, kp_t: float = 200.0,
                 ki_t: float = 400.0, tangent_ratio: float = 1.0, lead_m: float = 0.02, stall_s: float = 0.2,
                 stalled_m_s: float = 0.01, speed_filter_s: float = 0.1, finish_m: float = 0.005,
                 lift_speed_m_s: float = 0.03, done_m: float = 0.002, window: int = 6) -> None:
        self.device = torch.device(device)
        self.kp, self.ki, self.kp_t, self.ki_t, self.tangent_ratio = kp, ki, kp_t, ki_t, tangent_ratio
        self.lead_m, self.stall_s, self.stalled_m_s, self.speed_filter_s = lead_m, stall_s, stalled_m_s, speed_filter_s
        self.finish_m, self.lift_speed, self.done_m, self.window = finish_m, lift_speed_m_s, done_m, window
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        zeros3 = lambda: torch.zeros(num_envs, 3, device=self.device)
        flags = lambda: torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.origin, self.n_in, self.a, self.b = zeros3(), zeros3(), zeros3(), zeros3()
        self.points = torch.zeros(num_envs, PATH_SAMPLES, 2, device=self.device)
        self.arc = torch.zeros(num_envs, PATH_SAMPLES, device=self.device)
        self.standoff, self.overdrive, self.force_n = zeros(), zeros(), zeros()
        self.speed, self.approach_speed = zeros(), zeros()
        self.active = flags()
        #: State. `s` is the pad's depth past its start (the standoff off the estimated surface).
        self.s, self.v, self.ref, self.integral, self.stall = zeros(), zeros(), zeros(), zeros(), zeros()
        self.phase = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        #: The pad has stalled against the surface: the wipe along the path has begun.
        self.touched = flags()
        #: Along the path, m: the reference point, and how far the pad has got (monotone), and its nearest sample.
        self.path_ref, self.progress = zeros(), zeros()
        self.index = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.integral_t = zeros3()
        self.lift_ref = zeros()
        self.push = zeros()
        self.force_cmd = zeros3()
        #: The path's reference point and velocity, world (the velocity zero unless the reference is moving).
        self.ref_point = zeros3()
        self.ref_vel = zeros3()

    @property
    def total(self) -> torch.Tensor:
        return self.arc[:, -1]

    def start(self, env_ids: torch.Tensor, *, origin: torch.Tensor, normal: torch.Tensor, points: torch.Tensor,
              arc: torch.Tensor, standoff: torch.Tensor, overdrive: torch.Tensor, force_n: torch.Tensor,
              speed: torch.Tensor, approach_speed: torch.Tensor) -> None:
        """`origin` and `normal` (outward) are the estimated surface; `points` (count, PATH_SAMPLES, 2) and `arc` the
        path in its plane (`resample`)."""
        n = unit(normal)
        a, b = plane_axes(n)
        self.origin[env_ids], self.n_in[env_ids], self.a[env_ids], self.b[env_ids] = origin, -n, a, b
        self.points[env_ids], self.arc[env_ids] = points, arc
        self.standoff[env_ids], self.overdrive[env_ids], self.force_n[env_ids] = standoff, overdrive, force_n
        self.speed[env_ids], self.approach_speed[env_ids] = speed, approach_speed
        self.active[env_ids] = True
        for buf in (self.s, self.v, self.ref, self.integral, self.stall, self.path_ref, self.progress, self.lift_ref,
                    self.push):
            buf[env_ids] = 0.0
        self.phase[env_ids] = APPROACHING
        self.touched[env_ids] = False
        self.index[env_ids] = 0
        self.integral_t[env_ids] = 0.0
        self.force_cmd[env_ids] = 0.0
        self.ref_vel[env_ids] = 0.0

    def reset(self, env_ids: torch.Tensor) -> None:
        self.active[env_ids] = False
        self.force_cmd[env_ids] = 0.0
        self.push[env_ids] = 0.0
        self.integral[env_ids] = 0.0
        self.integral_t[env_ids] = 0.0
        self.phase[env_ids] = APPROACHING

    def to_world(self, point_ab: torch.Tensor) -> torch.Tensor:
        """A point in the estimated plane's (a, b) coordinates, on the estimated surface, world."""
        return self.origin + point_ab[:, :1] * self.a + point_ab[:, 1:] * self.b

    def in_plane(self, point: torch.Tensor) -> torch.Tensor:
        rel = point - self.origin
        return torch.stack(((rel * self.a).sum(-1), (rel * self.b).sum(-1)), dim=-1)

    def _measure_progress(self, pad_ab: torch.Tensor, wiping: torch.Tensor) -> None:
        """How far along the path the pad has got: the nearest sample within a window ahead of the last, then the
        projection onto the segment after it. Never goes back."""
        samples = PATH_SAMPLES
        offsets = torch.arange(-1, self.window + 1, device=self.device)
        cand = (self.index.unsqueeze(-1) + offsets).clamp(0, samples - 1)                  # (N, W)
        pts = self.points.gather(1, cand.unsqueeze(-1).expand(-1, -1, 2))
        best = (pts - pad_ab.unsqueeze(1)).norm(dim=-1).argmin(dim=-1)
        index = cand.gather(1, best.unsqueeze(-1)).squeeze(-1)
        index = torch.where(wiping, torch.maximum(index, self.index), self.index)
        nxt = (index + 1).clamp(max=samples - 1)
        p0 = self.points.gather(1, index.view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1)
        p1 = self.points.gather(1, nxt.view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1)
        seg = p1 - p0
        t = (((pad_ab - p0) * seg).sum(-1) / (seg * seg).sum(-1).clamp(min=1e-12)).clamp(0.0, 1.0)
        arc0, arc1 = self.arc.gather(1, index.unsqueeze(-1)).squeeze(-1), self.arc.gather(1, nxt.unsqueeze(-1)).squeeze(-1)
        progress = arc0 + t * (arc1 - arc0)
        self.index = index
        self.progress = torch.where(wiping, torch.maximum(self.progress, progress), self.progress)

    def step(self, pad: torch.Tensor, dt: float) -> tuple[torch.Tensor, torch.Tensor]:
        """(the pad's goal, the force command), world frame, from the pad centre's measured position."""
        active = self.active
        s = ((pad - self.origin) * self.n_in).sum(-1) + self.standoff
        raw = (s - self.s) / dt
        self.v = torch.where(active, self.v + (raw - self.v) * min(1.0, dt / self.speed_filter_s), self.v)
        self.s = torch.where(active, s, self.s)
        pushing = active & (self.phase <= WIPING)

        # Along the normal: a reference past the estimated surface, and the push capped at the wiping force.
        ref = torch.minimum(self.ref + self.approach_speed * dt, self.standoff + self.overdrive)
        self.ref = torch.where(pushing, ref, self.ref)
        lag = (self.ref - self.s) * pushing
        # Stalled against something while still asked to go further, pushing at the full force: the pad is on the
        # surface. Without the force a slow hand in free air reads the same, and the path starts before the contact.
        stalled = pushing & ~self.touched & (lag > 0.002) & (self.v.abs() < self.stalled_m_s) \
            & (self.push >= 0.9 * self.force_n)
        self.stall = torch.where(stalled, self.stall + dt, torch.zeros_like(self.stall))
        now_touched = self.stall > self.stall_s
        self.touched |= now_touched
        self.phase = torch.where(now_touched, torch.full_like(self.phase, WIPING), self.phase)
        integral = self.integral + lag * dt
        integral = torch.where(integral * lag < 0.0, torch.zeros_like(integral), integral)
        self.integral = torch.minimum(integral.clamp(min=0.0), self.force_n / self.ki) * pushing
        push = torch.minimum((self.kp * lag + self.ki * self.integral).clamp(min=0.0), self.force_n)

        # Along the path: the reference point runs at the wiping speed, kept within `lead_m` of the pad's progress.
        pad_ab = self.in_plane(pad)
        wiping = active & (self.phase == WIPING)
        self._measure_progress(pad_ab, wiping)
        advance = torch.minimum(torch.minimum(self.path_ref + self.speed * dt, self.progress + self.lead_m), self.total)
        moving = wiping & (advance > self.path_ref + 1e-9)
        self.path_ref = torch.where(wiping, advance, self.path_ref)
        goal_ab, tangent = point_at(self.points, self.arc, self.path_ref)
        tangent_w = tangent[:, :1] * self.a + tangent[:, 1:] * self.b
        self.ref_vel = torch.where(moving.unsqueeze(-1), self.speed.unsqueeze(-1) * tangent_w, torch.zeros_like(tangent_w))
        finished = wiping & (self.progress >= self.total - self.finish_m)
        self.phase = torch.where(finished, torch.full_like(self.phase, LIFTING), self.phase)
        self.lift_ref = torch.where(finished, self.s, self.lift_ref)

        # Across the normal while wiping: a PI force on the pad's miss of the reference point, which is what pushes
        # the pad through friction.
        miss_ab = goal_ab - pad_ab
        miss = miss_ab[:, :1] * self.a + miss_ab[:, 1:] * self.b
        cap_t = (self.tangent_ratio * self.force_n).unsqueeze(-1)
        integral_t = self.integral_t + miss * dt
        norm = integral_t.norm(dim=-1, keepdim=True)
        integral_t = integral_t * (cap_t / self.ki_t / norm.clamp(min=1e-9)).clamp(max=1.0)
        self.integral_t = integral_t * wiping.unsqueeze(-1)
        along = self.kp_t * miss + self.ki_t * self.integral_t
        along = along * (cap_t / along.norm(dim=-1, keepdim=True).clamp(min=1e-9)).clamp(max=1.0)
        along = along * wiping.unsqueeze(-1)

        # Lifting off at the end, back to the standoff.
        lifting = active & (self.phase == LIFTING)
        self.lift_ref = torch.where(lifting, (self.lift_ref - self.lift_speed * dt).clamp(min=0.0), self.lift_ref)
        out = lifting & (self.s <= self.done_m)
        self.phase = torch.where(out, torch.full_like(self.phase, DONE), self.phase)
        push = torch.where(self.phase <= WIPING, push, torch.zeros_like(push)) * active

        self.push = push
        self.force_cmd = push.unsqueeze(-1) * self.n_in + along
        depth = torch.where(self.phase <= WIPING, self.s,
                            torch.where(self.phase == LIFTING, self.lift_ref, torch.zeros_like(self.s)))
        self.ref_point = self.to_world(goal_ab)
        goal = self.ref_point + (depth - self.standoff).unsqueeze(-1) * self.n_in
        return goal, self.force_cmd

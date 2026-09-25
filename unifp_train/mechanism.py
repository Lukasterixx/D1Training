"""A one-degree-of-freedom mechanism the robot has hold of: a drawer, a latch, a door, a button.

This is the plant of the goal-commanded task (`mech_env.py`). The robot already has the handle --
the claw is on it, the task starts there -- and is told only where the handle should go. How hard
that is, it is not told: the mechanism's spring, friction and latch are drawn per episode and never
observed, so the force is the policy's to find, and when the arm alone cannot make it, the body has
to.

**The path.** The handle moves on one curve, parameterised by `s`, metres along it from closed:

  * a *slide* -- drawer, bolt, latch, button -- runs straight: ``p(s) = start + s d``;
  * a *hinge* -- door, lid -- swings about an axis ``a`` through ``centre``:
    ``p(s) = centre + cos(s/r) u + sin(s/r) (a x u)``, with ``u = start - centre`` and ``r = |u|``,
    so `s` is arc length at the handle and every force below is in newtons at the handle.

**The resistance**, all along the path, positive resisting opening:

  * a spring toward closed, ``spring_n + spring_k s`` (a closer, a return spring);
  * viscous damping ``damping v``;
  * Coulomb friction: nothing moves until the net force passes ``friction_static``; once moving,
    ``friction_kinetic`` opposes the motion;
  * a latch: at closed, the mechanism does not open until the net opening force passes
    ``friction_static + latch_n`` -- then it lets go at once, which is the snap of a latch or an
    emergency-stop button, and whatever force the robot had built up is suddenly unopposed. It
    catches again whenever the mechanism is closed;
  * hard stops at 0 and ``travel``.

**The grasp** is a stiff 3-D spring-damper between the tool point and the handle point. Along the
path it drives the mechanism; across it, the mechanism is rigid and the spring is what the robot
feels as the hinge or the rails. A spring force above ``grip_n`` tears the handle out of the claw.
There is no torque through the grasp -- a ball joint -- so the wrist is free; a real handle would
constrain it.

Stepped at the physics rate, like `fixture.ContactFixture`: the grasp is a few thousand N/m, and a
force updated every 20 ms would ring. No Isaac imports (`tests/test_mechanism.py`).
"""
from __future__ import annotations

import math

import torch

#: Below this handle speed, m/s, the mechanism counts as stopped and stiction applies.
STOPPED_M_S = 1.0e-3


class Mechanism:
    """One mechanism per environment, grasped on demand."""

    def __init__(self, num_envs: int, device, *, grasp_damping: float) -> None:
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.grasp_damping = grasp_damping
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        zeros3 = lambda: torch.zeros(num_envs, 3, device=self.device)
        flags = lambda: torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.hinge = flags()
        self.start = zeros3()       # the handle at s = 0, env frame
        self.direction = zeros3()   # a slide's unit opening direction
        self.centre = zeros3()      # a hinge's axis passes through here
        self.axis = zeros3()        # a hinge's unit axis, oriented so +s opens
        self.radial = zeros3()      # u = start - centre, perpendicular to the axis
        self.radius = zeros()
        self.travel = zeros()
        self.mass = torch.ones(num_envs, device=self.device)
        self.spring_n = zeros()
        self.spring_k = zeros()
        self.damping = zeros()
        self.friction_static = zeros()
        self.friction_kinetic = zeros()
        self.latch_n = zeros()
        self.grasp_k = zeros()
        self.grip_n = torch.full((num_envs,), 1.0e9, device=self.device)
        #: State.
        self.s = zeros()
        self.v = zeros()
        self.latched = flags()
        self.grasped = flags()
        #: Set on the physics step the handle was torn out of the claw; cleared on reset.
        self.torn = flags()
        #: True once the latch has let go this episode, and the step count at which it first did.
        self.released = flags()
        #: Force on the tool from the grasp, world frame, as of the last `step()`.
        self.force_on_tool = zeros3()
        #: The force the robot drives the mechanism with along its path, N, as of the last `step()`.
        self.drive_n = zeros()

    # --- geometry ------------------------------------------------------------------------------

    def position(self, s: torch.Tensor | None = None) -> torch.Tensor:
        """The handle point at `s` (default: where it is), env frame, (N, 3)."""
        return path_position(self.hinge, self.start, self.direction, self.centre, self.axis, self.radial,
                             self.radius, self.s if s is None else s)

    def tangent(self, s: torch.Tensor | None = None) -> torch.Tensor:
        """Unit direction of increasing `s` at `s`, (N, 3)."""
        s = self.s if s is None else s
        angle = (s / self.radius.clamp(min=1e-6)).unsqueeze(-1)
        swing = (-torch.sin(angle) * self.radial
                 + torch.cos(angle) * torch.cross(self.axis, self.radial, dim=-1)) / self.radius.clamp(min=1e-6).unsqueeze(-1)
        return torch.where(self.hinge.unsqueeze(-1), swing, self.direction)

    # --- engaging and resetting -----------------------------------------------------------------

    def grasp(self, env_ids: torch.Tensor, point: torch.Tensor, *, hinge: torch.Tensor,
              opening: torch.Tensor, axis: torch.Tensor, radius: torch.Tensor, travel: torch.Tensor,
              mass: torch.Tensor, spring_n: torch.Tensor, spring_k: torch.Tensor, damping: torch.Tensor,
              friction_static: torch.Tensor, friction_kinetic: torch.Tensor, latch_n: torch.Tensor,
              grasp_k: torch.Tensor, grip_n: torch.Tensor) -> None:
        """Take hold of a mechanism whose handle is at `point` (env frame), closed.

        `opening` is the unit direction the handle first moves in as it opens. For a hinge, `axis`
        must be perpendicular to it, and the hinge is put `radius` away so that it does: the centre
        is ``point - radius (opening x axis)``, which makes the swing's tangent at closed `opening`.
        """
        opening = opening / opening.norm(dim=-1, keepdim=True)
        axis = axis / axis.norm(dim=-1, keepdim=True).clamp(min=1e-9)
        radial, centre = hinge_geometry(point, opening, axis, radius)
        self.hinge[env_ids] = hinge
        self.start[env_ids] = point
        self.direction[env_ids] = opening
        self.axis[env_ids] = axis
        self.radial[env_ids] = radial
        self.centre[env_ids] = centre
        self.radius[env_ids] = torch.where(hinge, radius, torch.ones_like(radius))
        self.travel[env_ids] = travel
        self.mass[env_ids] = mass
        self.spring_n[env_ids] = spring_n
        self.spring_k[env_ids] = spring_k
        self.damping[env_ids] = damping
        self.friction_static[env_ids] = friction_static
        self.friction_kinetic[env_ids] = torch.minimum(friction_kinetic, friction_static)
        self.latch_n[env_ids] = latch_n
        self.grasp_k[env_ids] = grasp_k
        self.grip_n[env_ids] = grip_n
        self.s[env_ids] = 0.0
        self.v[env_ids] = 0.0
        self.latched[env_ids] = latch_n > 0.0
        self.released[env_ids] = False
        self.grasped[env_ids] = True
        self.torn[env_ids] = False

    def reset(self, env_ids: torch.Tensor) -> None:
        self.grasped[env_ids] = False
        self.torn[env_ids] = False
        self.latched[env_ids] = False
        self.released[env_ids] = False
        self.s[env_ids] = 0.0
        self.v[env_ids] = 0.0
        self.force_on_tool[env_ids] = 0.0
        self.drive_n[env_ids] = 0.0

    # --- the step --------------------------------------------------------------------------------

    def resistance_at(self, s: torch.Tensor) -> torch.Tensor:
        """The spring's force toward closed at `s`, N."""
        return self.spring_n + self.spring_k * s

    def step(self, tool_pos: torch.Tensor, tool_vel: torch.Tensor, dt: float) -> torch.Tensor:
        """Advance one physics step; return the force on the tool (N, 3), world frame.

        `tool_pos` is in the env frame (the mechanism's), `tool_vel` in the world frame. The grasp
        force is computed from the state at the start of the step, then the mechanism is advanced
        under it (semi-implicit Euler, with the stick-slip decided before the velocity update).
        """
        handle = self.position()
        tangent = self.tangent()
        stretch = tool_pos - handle
        relative = tool_vel - self.v.unsqueeze(-1) * tangent
        spring = -self.grasp_k.unsqueeze(-1) * stretch
        on_tool = spring - self.grasp_damping * relative
        # What drives the mechanism along its path is the reaction, on the handle.
        drive = -(on_tool * tangent).sum(-1)
        net = drive - self.resistance_at(self.s) - self.damping * self.v

        moving = self.v.abs() > STOPPED_M_S
        hold_open = self.friction_static + self.latched * self.latch_n
        at_closed, at_open = self.s <= 0.0, self.s >= self.travel
        breaks_open = ~moving & (net > hold_open) & ~at_open
        breaks_closed = ~moving & (net < -self.friction_static) & ~at_closed
        free = moving | breaks_open | breaks_closed
        # The latch lets go the moment the mechanism starts to open.
        let_go = breaks_open & self.latched & self.grasped
        self.released |= let_go
        self.latched &= ~let_go

        direction = torch.where(moving, torch.sign(self.v), torch.sign(net))
        accel = (net - direction * self.friction_kinetic) / self.mass
        v_new = torch.where(free, self.v + accel * dt, torch.zeros_like(self.v))
        # Friction stops a motion; it never reverses one.
        reversed_ = moving & (torch.sign(v_new) != torch.sign(self.v)) & (net.abs() <= self.friction_static)
        v_new = torch.where(reversed_, torch.zeros_like(v_new), v_new)
        s_new = self.s + v_new * dt
        hit_closed, hit_open = s_new <= 0.0, s_new >= self.travel
        s_new = torch.minimum(torch.maximum(s_new, torch.zeros_like(s_new)), self.travel)
        v_new = torch.where((hit_closed & (v_new < 0.0)) | (hit_open & (v_new > 0.0)), torch.zeros_like(v_new), v_new)
        # A latch catches again when the mechanism is shut.
        self.latched |= hit_closed & (self.latch_n > 0.0)

        active = self.grasped
        self.s = torch.where(active, s_new, self.s)
        self.v = torch.where(active, v_new, self.v)
        torn = active & (spring.norm(dim=-1) > self.grip_n)
        self.torn |= torn
        self.grasped &= ~torn
        self.force_on_tool = torch.where(self.grasped.unsqueeze(-1), on_tool, torch.zeros_like(on_tool))
        self.drive_n = torch.where(self.grasped, drive, torch.zeros_like(drive))
        return self.force_on_tool

    def applied_by_robot(self) -> torch.Tensor:
        """The force the robot applies to the handle: the reaction to `force_on_tool`."""
        return -self.force_on_tool

    def peak_resistance(self) -> torch.Tensor:
        """The largest force opening fully needs from rest, N: at closed (spring, stiction and latch)
        or at the far stop (spring and stiction), whichever is more."""
        return torch.maximum(self.spring_n + self.friction_static + self.latch_n,
                             self.resistance_at(self.travel) + self.friction_static)


def hinge_geometry(point: torch.Tensor, opening: torch.Tensor, axis: torch.Tensor,
                   radius: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """(u, centre) of a hinge whose handle at `point` starts out moving along unit `opening`.

    ``u = radius (opening x axis)`` and ``centre = point - u``: then ``axis x u = radius opening``
    (the axis is perpendicular to the opening), so the swing's tangent at closed is `opening`.
    """
    radial = radius.unsqueeze(-1) * torch.cross(opening, axis, dim=-1)
    return radial, point - radial


def path_position(hinge: torch.Tensor, start: torch.Tensor, direction: torch.Tensor, centre: torch.Tensor,
                  axis: torch.Tensor, radial: torch.Tensor, radius: torch.Tensor, s: torch.Tensor) -> torch.Tensor:
    """The handle point `s` metres along a slide (``start + s direction``) or a hinge's arc."""
    slide = start + s.unsqueeze(-1) * direction
    angle = (s / radius.clamp(min=1e-6)).unsqueeze(-1)
    swing = centre + torch.cos(angle) * radial + torch.sin(angle) * torch.cross(axis, radial, dim=-1)
    return torch.where(hinge.unsqueeze(-1), swing, slide)


def resistance_profile(peak_n: torch.Tensor, *, latch: torch.Tensor, preload_fraction: torch.Tensor,
                       kinetic_fraction: torch.Tensor, weights: torch.Tensor) -> dict[str, torch.Tensor]:
    """Split a required peak force into a spring, friction and a latch whose peak is exactly `peak_n`.

    `weights` (N, 3) are the relative shares of the spring (at full travel), stiction and the latch;
    the latch's share is dropped where `latch` is False. The preload is `preload_fraction` of the
    spring's full-travel force. The two peaks -- at closed, ``preload + stiction + latch``, and at
    full travel, ``spring + stiction`` -- are then scaled together so the larger is `peak_n`.
    Returns the spring's force at closed and at full travel rather than a rate, so the caller divides
    by whatever travel the path ends up with.
    """
    w = weights.clone()
    w[:, 2] = torch.where(latch, w[:, 2], torch.zeros_like(w[:, 2]))
    w = w / w.sum(-1, keepdim=True).clamp(min=1e-9)
    spring_end, static, latch_n = w[:, 0], w[:, 1], w[:, 2]
    preload = preload_fraction * spring_end
    peak = torch.maximum(preload + static + latch_n, spring_end + static).clamp(min=1e-9)
    scale = peak_n / peak
    return {"spring_closed_n": preload * scale, "spring_open_n": spring_end * scale,
            "friction_static": static * scale, "friction_kinetic": static * scale * kinetic_fraction,
            "latch_n": latch_n * scale}


def hinge_axes(opening: torch.Tensor, vertical: torch.Tensor, flip: torch.Tensor) -> torch.Tensor:
    """A hinge axis perpendicular to each opening direction: vertical (a door) or horizontal (a lid).

    `vertical` picks which; where the opening is itself near vertical, a vertical axis does not exist
    and the horizontal one is used. `flip` reverses the axis, which puts the hinge on the other side.
    """
    up = torch.zeros_like(opening)
    up[:, 2] = 1.0
    upright = up - (up * opening).sum(-1, keepdim=True) * opening
    level = torch.cross(opening, up, dim=-1)
    level = torch.where(level.norm(dim=-1, keepdim=True) < 1e-6,
                        torch.tensor([0.0, 1.0, 0.0], device=opening.device).expand_as(opening), level)
    use_upright = vertical & (upright.norm(dim=-1) > 0.3)
    axis = torch.where(use_upright.unsqueeze(-1), upright, level)
    axis = axis / axis.norm(dim=-1, keepdim=True)
    return torch.where(flip.unsqueeze(-1), -axis, axis)


class TravelGoal:
    """Where the handle is asked to be: a reference sliding to a target, holding, then another.

    The first target of an episode opens the mechanism most of the way (`first_fraction` of its
    travel); later ones are anywhere along it, closing included. The reference moves at `speed`
    m/s, drawn per episode -- a task layer that knows the geometry (from tags) and plans along it,
    but knows nothing of the resistance. A target is held for a drawn time once the reference is on
    it, then the next is drawn.

    Evaluation replaces the draws with a fixed plan (`use_plan`): one target, held to the end.
    """

    def __init__(self, num_envs: int, device, *, speed_m_s: tuple[float, float], hold_s: tuple[float, float],
                 first_fraction: tuple[float, float], dt: float, generator: torch.Generator | None = None) -> None:
        self.device = torch.device(device)
        self.dt = dt
        self.speed_range = speed_m_s
        self.hold_steps = (math.ceil(hold_s[0] / dt), math.ceil(hold_s[1] / dt))
        self.first_fraction = first_fraction
        self.generator = generator
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        self.ref = zeros()
        self.target = zeros()
        #: The first target of the episode: what "opened" is measured against.
        self.first_target = zeros()
        self.speed = zeros()
        self.timer = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.plan: tuple[float, float] | None = None   # (fraction of travel, speed m/s)

    def _rand(self, count: int) -> torch.Tensor:
        return torch.rand(count, device=self.device, generator=self.generator)

    def use_plan(self, fraction: float, speed_m_s: float) -> None:
        self.plan = (float(fraction), float(speed_m_s))

    def _hold(self, count: int) -> torch.Tensor:
        low, high = self.hold_steps
        return low + (self._rand(count) * (high - low)).long()

    def start(self, env_ids: torch.Tensor, travel: torch.Tensor) -> None:
        count = len(env_ids)
        self.ref[env_ids] = 0.0
        if self.plan is not None:
            fraction = torch.full((count,), self.plan[0], device=self.device)
            self.speed[env_ids] = self.plan[1]
            self.timer[env_ids] = 10 ** 9
        else:
            low, high = self.first_fraction
            fraction = low + self._rand(count) * (high - low)
            low, high = self.speed_range
            self.speed[env_ids] = low + self._rand(count) * (high - low)
            self.timer[env_ids] = self._hold(count)
        self.target[env_ids] = fraction * travel
        self.first_target[env_ids] = self.target[env_ids]

    def reset(self, env_ids: torch.Tensor) -> None:
        self.ref[env_ids] = 0.0
        self.target[env_ids] = 0.0
        self.first_target[env_ids] = 0.0
        self.speed[env_ids] = 0.0
        self.timer[env_ids] = 0

    def step(self, active: torch.Tensor, travel: torch.Tensor) -> torch.Tensor:
        """Advance one policy step for the `active` environments and return the reference, m."""
        change = (self.target - self.ref).clamp(-1.0, 1.0)
        change = torch.maximum(torch.minimum(change, self.speed * self.dt), -self.speed * self.dt)
        self.ref = torch.where(active, self.ref + change, self.ref)
        arrived = active & ((self.target - self.ref).abs() < 1e-6)
        self.timer = torch.where(arrived, self.timer - 1, self.timer)
        due = arrived & (self.timer <= 0)
        if bool(due.any()):
            ids = due.nonzero(as_tuple=False).flatten()
            self.target[ids] = self._rand(len(ids)) * travel[ids]
            self.timer[ids] = self._hold(len(ids))
        return self.ref

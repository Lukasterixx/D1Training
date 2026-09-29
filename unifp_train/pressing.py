"""Pressing a spring-loaded push button: the tools, the plant, and the task layer that drives the press.

The plan's force-calibration task (`docs/thesis_b_plan.md`, "Contact-task progression"): a spring-loaded button with
travel, spring and damping; the bare end effector, then a rounded probe; success is the button's full travel held for
1 s, reported with the force error and peak, the time and the failure reason. Built as the key insertion is
(`insertion.py`): an analytic plant stepped at the physics rate, so a training run can hold thousands of them, and a
task layer that drives the press from an estimate of where the button is.

**The tool** ends in a sphere: its far point `length_m` beyond the jaw centre along the hand's approach, its radius
`radius_m` (`TOOLS`). The bare end effector is the closed jaws' end faces, 20 mm beyond the jaw centre
(`demos/cup/pick_demo/grasp.FINGERTIP_Z_M`), modelled as a 10 mm sphere; the probe is a 3 mm-radius ball 70 mm out,
a rod held in the jaws as the key is. A sphere meets a plane the same way at any tilt, so the hand's attitude does not
enter the contact; the approach is commanded along the button's axis all the same, as a rod would want.

**The button** is a round cap of radius `cap_radius` on the axis `n` (into the panel), its face standing `proud` in
front of the panel's face (negative: recessed), in a hole of the same radius. Pressed in by `s`, it resists with
`preload + stiffness s + snap(s)` plus `damping` times its speed, and stops at `travel`. `snap(s)` is a tactile
button's click: an extra force rising to `snap_n` over `SNAP_WIDTH_M` before `snap_at` of the travel and gone past it,
re-armed as the cap returns. The cap is massless: it sits where the tool's contact spring and the button balance (a few
fixed-point passes per step, for the snap), so it follows a withdrawing tool back out and returns when left.

**The contact** is a penalty model on the tool's sphere, every physics step:

  * over the cap (the sphere's centre within `cap_radius` of the axis), a spring on its overlap with the cap's face;
  * elsewhere, the panel's face, `proud` behind the cap's rest face -- a tool that misses the cap presses the panel;
  * over the cap, also the rim of the hole in the panel (and the hole's wall past it): a sphere wider than the hole
    cannot follow a recessed cap in. This is what lets a probe press a small recessed button the bare jaws cannot --
    an access limitation, which the plan asks to be declared as one rather than read as worse control;
  * Coulomb friction against sliding across whatever it presses, ramped below `insertion.FRICTION_V_EPS`.

Everything is a force on the tool, in the world frame; the environment applies it at the tool's tip.

**The task layer** (`PressTaskLayer`) is what a robot would run, knowing the button only from an estimate and a
datasheet: the tip to a standoff short of the estimated cap; then a PI force law along the estimated axis on how far
the tip lags a reference depth running past the estimated full travel -- capped at a light `touch_n` until the tip has
stalled on something, at `max_n` after; at full travel (by the estimate, or stalled at the cap) a steady `hold_n` for
`hold_s`; then back out to the standoff. Only the tip's position is read.

Plain torch, no Isaac imports (`tests/test_pressing.py`). Every number is invented or taken from common pushbutton
datasheets; none is measured.
"""
from __future__ import annotations

from dataclasses import dataclass

import torch

from .insertion import FRICTION_V_EPS, unit

#: Penalty contact: the key insertion's stiffness and damping (`insertion.CONTACT_K_N_PER_M`), stable at 5 ms.
CONTACT_K_N_PER_M = 16000.0
CONTACT_C_N_S_PER_M = 50.0
#: The click's width before the point it lets go.
SNAP_WIDTH_M = 0.001
#: Fixed-point passes for the cap's position each step. The snap's slope (at most ~8,000 N/m) is below the contact's
#: 16,000, so each pass at least halves the error.
SOLVE_ITERATIONS = 4
#: The cap within this of its full travel is pressed.
PRESSED_TOL_M = 0.0005
#: The panel extends this far from the axis; a tool this far into its face has gone through it (damage), and no
#: penalty force grows past `MAX_OVER_M` of overlap, so a hand that starts somewhere unphysical is not flung.
PANEL_HALF_M = 0.15
FACE_DEPTH_M = 0.01
MAX_OVER_M = 0.01


@dataclass(frozen=True)
class Tool:
    """A round-ended pressing tool: its far point `length_m` beyond the jaw centre along the approach."""

    name: str
    length_m: float
    radius_m: float


TOOLS = {
    # The closed jaws' end faces (grasp.FINGERTIP_Z_M, 20 mm past the jaw centre), as a 10 mm sphere. A model: the
    # real end faces are two flat 12 mm fingers side by side.
    "bare": Tool("bare", 0.020, 0.010),
    # A rod held in the jaws with a 6 mm ball, reaching as far as the key does.
    "probe": Tool("probe", 0.070, 0.003),
}
TOOL_NAMES = tuple(TOOLS)


def reachable_front(radius: torch.Tensor, cap_radius: torch.Tensor, proud: torch.Tensor) -> torch.Tensor:
    """How deep past the cap's rest face a sphere centred on the axis can put its front before the hole's rim stops it.

    A sphere no wider than the hole goes in (unlimited, returned as +inf); a wider one rests on the rim with its centre
    `sqrt(radius^2 - cap_radius^2)` in front of the panel's face."""
    wide = radius > cap_radius
    rest = proud - torch.sqrt((radius ** 2 - cap_radius ** 2).clamp(min=0.0)) + radius
    return torch.where(wide, rest, torch.full_like(rest, float("inf")))


class Button:
    """One spring-loaded push button per environment, and a round-ended tool's contact with it."""

    def __init__(self, num_envs: int, device, *, k: float = CONTACT_K_N_PER_M, c: float = CONTACT_C_N_S_PER_M) -> None:
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.k, self.c = k, c
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        zeros3 = lambda: torch.zeros(num_envs, 3, device=self.device)
        flags = lambda: torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.face, self.n = zeros3(), zeros3()
        self.cap_radius, self.proud, self.travel = zeros(), zeros(), zeros()
        self.preload, self.stiffness, self.damping = zeros(), zeros(), zeros()
        self.snap_n, self.snap_at, self.mu = zeros(), zeros(), zeros()
        self.active = flags()
        #: State: how far the cap is pressed in; the tool on the cap now, and ever; driven through the panel.
        self.s = zeros()
        self.on_cap = flags()
        self.touched_cap = flags()
        self.pierced = flags()
        #: As of the last `step()`: the force on the tool (world), every contact's normal force summed, the force along
        #: -n (pushing the tool back out), the force across the axis, and the tool's distance from the axis.
        self.force_on_tool = zeros3()
        self.normal_n = zeros()
        self.axial_n = zeros()
        self.lateral_n = zeros()
        self.off_axis_m = zeros()
        #: The largest summed normal force since `take_peak()`: the physics-rate peak a 50 Hz sample would miss.
        self.peak_n = zeros()

    def place(self, env_ids: torch.Tensor, *, face: torch.Tensor, axis: torch.Tensor, cap_radius: torch.Tensor,
              proud: torch.Tensor, travel: torch.Tensor, preload_n: torch.Tensor, stiffness: torch.Tensor,
              damping: torch.Tensor, snap_n: torch.Tensor, snap_at: torch.Tensor, mu: torch.Tensor) -> None:
        self.face[env_ids], self.n[env_ids] = face, unit(axis)
        self.cap_radius[env_ids], self.proud[env_ids], self.travel[env_ids] = cap_radius, proud, travel
        self.preload[env_ids], self.stiffness[env_ids], self.damping[env_ids] = preload_n, stiffness, damping
        self.snap_n[env_ids], self.snap_at[env_ids], self.mu[env_ids] = snap_n, snap_at, mu
        self.active[env_ids] = True
        self._clear(env_ids)

    def reset(self, env_ids: torch.Tensor) -> None:
        self.active[env_ids] = False
        self._clear(env_ids)

    def _clear(self, env_ids: torch.Tensor) -> None:
        self.s[env_ids] = 0.0
        for flag in (self.on_cap, self.touched_cap, self.pierced):
            flag[env_ids] = False
        self.force_on_tool[env_ids] = 0.0
        for buf in (self.normal_n, self.axial_n, self.lateral_n, self.off_axis_m, self.peak_n):
            buf[env_ids] = 0.0

    # --- the button's own force ---------------------------------------------------------------------------------

    def snap(self, s: torch.Tensor) -> torch.Tensor:
        """The click's extra resistance at `s`: rising over `SNAP_WIDTH_M`, gone past `snap_at` of the travel."""
        release = self.snap_at * self.travel
        rise = ((s - (release - SNAP_WIDTH_M)) / SNAP_WIDTH_M).clamp(0.0, 1.0)
        return self.snap_n * rise * (s < release)

    def resistance(self, s: torch.Tensor) -> torch.Tensor:
        """The button's static force at `s`, N."""
        return self.preload + self.stiffness * s + self.snap(s)

    def full_travel_n(self) -> torch.Tensor:
        """What it takes to hold the button at its full travel, N (the click is past by then)."""
        return self.preload + self.stiffness * self.travel

    def operating_n(self) -> torch.Tensor:
        """The most it resists anywhere on its travel -- the click's peak, or the full travel -- N: a datasheet's
        operating force."""
        before_click = self.preload + self.stiffness * (self.snap_at * self.travel) + self.snap_n
        return torch.maximum(before_click, self.full_travel_n())

    def pressed(self, tolerance_m: float = PRESSED_TOL_M) -> torch.Tensor:
        return self.active & (self.s >= self.travel - tolerance_m)

    def take_peak(self) -> torch.Tensor:
        """The physics-rate peak normal force since the last call, and start a new one."""
        peak = self.peak_n.clone()
        self.peak_n.zero_()
        return peak

    # --- the contact --------------------------------------------------------------------------------------------

    def step(self, centre: torch.Tensor, vel: torch.Tensor, radius: torch.Tensor, dt: float) -> torch.Tensor:
        """The force on the tool (world, (N, 3)) with its sphere of `radius` centred at `centre`, moving at `vel`.
        Updates the cap's position and the contact state."""
        active = self.active
        q = centre - self.face
        d = (q * self.n).sum(-1)                        # the sphere's centre past the cap's rest face
        lat = q - d.unsqueeze(-1) * self.n
        e = lat.norm(dim=-1)
        lat_dir = lat / e.clamp(min=1e-9).unsqueeze(-1) * (e > 1e-6).unsqueeze(-1)
        front = d + radius
        v_n = (vel * self.n).sum(-1)
        v_lat = vel - v_n.unsqueeze(-1) * self.n
        over_cap = active & (e < self.cap_radius)

        # The cap: where the contact spring and the button balance, K (front - s) = resistance(s), within its travel.
        s = self.s
        for _ in range(SOLVE_ITERATIONS):
            s = ((self.k * front - self.preload - self.snap(s)) / (self.k + self.stiffness)).clamp(min=0.0)
            s = torch.minimum(s, self.travel)
        s = torch.where(over_cap, s, torch.zeros_like(s))
        s_dot = (s - self.s) / dt
        overlap_cap = (front - s).clamp(min=0.0, max=MAX_OVER_M) * over_cap
        on_cap = overlap_cap > 0.0
        moving = (s > 0.0) & (s < self.travel)
        cap_n = (self.k * overlap_cap + self.c * (v_n - s_dot).clamp(min=0.0) + self.damping * s_dot * moving) * on_cap
        cap_n = cap_n.clamp(min=0.0)

        # The panel's face, `proud` behind the cap's rest face, where the tool is not over the cap.
        on_panel = active & ~over_cap & (e < PANEL_HALF_M)
        into_panel = (front - self.proud).clamp(min=0.0) * on_panel
        pierced = self.pierced | (into_panel >= FACE_DEPTH_M)
        overlap_panel = into_panel.clamp(max=MAX_OVER_M)
        panel_n = (self.k * overlap_panel + self.c * v_n.clamp(min=0.0) * (overlap_panel > 0.0))

        # Over the cap, the hole's rim (the sphere's centre in front of the panel's face) or its wall (past it).
        dz = d - self.proud
        dr = e - self.cap_radius
        dist = torch.sqrt(dz ** 2 + dr ** 2).clamp(min=1e-9)
        in_front = dz < 0.0
        rim_over = (radius - dist).clamp(min=0.0, max=MAX_OVER_M) * over_cap * in_front
        rim_dir = (dz / dist).unsqueeze(-1) * self.n + (dr / dist).unsqueeze(-1) * lat_dir
        wall_over = (e + radius - self.cap_radius).clamp(min=0.0, max=MAX_OVER_M) * over_cap * ~in_front
        rim = self.k * rim_over.unsqueeze(-1) * rim_dir - (self.k * wall_over).unsqueeze(-1) * lat_dir

        # Friction across the axis, against the tool's sliding, from the faces it presses.
        face_n = cap_n + panel_n
        speed = v_lat.norm(dim=-1, keepdim=True)
        friction = -(self.mu * face_n).unsqueeze(-1) * v_lat / speed.clamp(min=1e-9) * torch.tanh(speed / FRICTION_V_EPS)

        force = -face_n.unsqueeze(-1) * self.n + rim + friction
        force = torch.where(active.unsqueeze(-1), force, torch.zeros_like(force))
        normal = (face_n + self.k * (rim_over + wall_over)) * active

        self.s = torch.where(active, s, self.s)
        self.on_cap = on_cap
        self.touched_cap |= on_cap
        self.pierced = pierced
        self.force_on_tool = force
        self.normal_n = normal
        along = (force * self.n).sum(-1)
        self.axial_n = (-along).clamp(min=0.0) * active
        self.lateral_n = (force - along.unsqueeze(-1) * self.n).norm(dim=-1) * active
        self.off_axis_m = e * active
        self.peak_n = torch.maximum(self.peak_n, normal)
        return force


#: `PressTaskLayer.phase`.
PRESSING, HOLDING, RELEASING, DONE = 0, 1, 2, 3


class PressTaskLayer:
    """The press as a robot would run it, from an estimate of the button and its datasheet, and the tip's position.

    Per environment: the estimated cap face and axis, the travel, the forces (`touch_n`, `max_n`, `hold_n`), a
    reference depth that runs from the standoff to `overdrive` past the estimated full travel at a set speed, and a PI
    law along the axis on how far the tip lags it. The goal it returns is for the tool's tip: along the axis where the
    tip is while it pushes (the law carries the push), back along the axis while it releases; across the axis on the
    estimate. The force is along the estimated axis.
    """

    def __init__(self, num_envs: int, device, *, kp: float = 400.0, ki: float = 1000.0, stall_s: float = 0.2,
                 stalled_m_s: float = 0.01, lead_m: float = 0.003, speed_filter_s: float = 0.1,
                 bottom_tol_m: float = 0.001, release_speed_m_s: float = 0.03, done_m: float = 0.002) -> None:
        self.device = torch.device(device)
        self.kp, self.ki, self.stall_s, self.stalled_m_s = kp, ki, stall_s, stalled_m_s
        self.lead_m, self.speed_filter_s, self.bottom_tol_m = lead_m, speed_filter_s, bottom_tol_m
        self.release_speed, self.done_m = release_speed_m_s, done_m
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        zeros3 = lambda: torch.zeros(num_envs, 3, device=self.device)
        flags = lambda: torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.face, self.n = zeros3(), zeros3()
        self.standoff, self.travel, self.overdrive, self.speed = zeros(), zeros(), zeros(), zeros()
        self.touch_n, self.max_n, self.hold_n, self.hold_s = zeros(), zeros(), zeros(), zeros()
        self.active = flags()
        #: State. `s` is the tip's depth past its start (the standoff short of the estimated face).
        self.s, self.v, self.ref, self.integral, self.stall = zeros(), zeros(), zeros(), zeros(), zeros()
        self.phase = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        #: The tip has stalled on something at the light touch cap: from then on the law may push to `max_n`.
        self.touched = flags()
        #: Time spent holding, and the release's reference depth.
        self.held_s, self.release_ref = zeros(), zeros()
        self.push = zeros()
        self.force_cmd = zeros3()

    def start(self, env_ids: torch.Tensor, *, face: torch.Tensor, axis: torch.Tensor, standoff: torch.Tensor,
              travel: torch.Tensor, overdrive: torch.Tensor, speed: torch.Tensor, touch_n: torch.Tensor,
              max_n: torch.Tensor, hold_n: torch.Tensor, hold_s: torch.Tensor) -> None:
        self.face[env_ids], self.n[env_ids] = face, unit(axis)
        self.standoff[env_ids], self.travel[env_ids], self.overdrive[env_ids] = standoff, travel, overdrive
        self.speed[env_ids], self.touch_n[env_ids], self.max_n[env_ids] = speed, touch_n, max_n
        self.hold_n[env_ids], self.hold_s[env_ids] = hold_n, hold_s
        self.active[env_ids] = True
        for buf in (self.s, self.v, self.ref, self.integral, self.stall, self.held_s, self.release_ref, self.push):
            buf[env_ids] = 0.0
        self.phase[env_ids] = PRESSING
        self.touched[env_ids] = False
        self.force_cmd[env_ids] = 0.0

    def reset(self, env_ids: torch.Tensor) -> None:
        self.active[env_ids] = False
        self.force_cmd[env_ids] = 0.0
        self.push[env_ids] = 0.0
        self.integral[env_ids] = 0.0
        self.phase[env_ids] = PRESSING

    @property
    def bottom(self) -> torch.Tensor:
        """The estimated full travel, as a depth past the start."""
        return self.standoff + self.travel

    def cap(self) -> torch.Tensor:
        return torch.where(self.touched, self.max_n, self.touch_n)

    def step(self, tip: torch.Tensor, dt: float) -> tuple[torch.Tensor, torch.Tensor]:
        """(the tip's goal, the force command), world frame, from the tip's measured position."""
        active = self.active
        s = ((tip - self.face) * self.n).sum(-1) + self.standoff
        raw = (s - self.s) / dt
        self.v = torch.where(active, self.v + (raw - self.v) * min(1.0, dt / self.speed_filter_s), self.v)
        self.s = torch.where(active, s, self.s)
        pressing = active & (self.phase == PRESSING)

        # The reference runs past the estimated full travel whatever the tip does.
        ref = torch.minimum(self.ref + self.speed * dt, self.bottom + self.overdrive)
        self.ref = torch.where(pressing, ref, self.ref)
        lag = (self.ref - self.s) * pressing
        cap = self.cap()
        # Stalled: asked to go further, pushing at the cap, and not moving.
        stalled = pressing & (lag > 0.5 * self.lead_m) & (self.v.abs() < self.stalled_m_s) & (self.push >= 0.9 * cap)
        self.stall = torch.where(stalled, self.stall + dt, torch.zeros_like(self.stall))
        stuck = self.stall > self.stall_s
        # Stalled at the light cap: something is touched. Raise the cap, bring the reference back to just ahead of the
        # tip and clear the integral, so what built up while waiting does not lunge the cap in.
        now_touched = stuck & ~self.touched
        self.touched |= now_touched
        self.ref = torch.where(now_touched, torch.minimum(self.ref, self.s + self.lead_m), self.ref)
        self.integral = torch.where(now_touched, torch.zeros_like(self.integral), self.integral)
        self.stall = torch.where(now_touched, torch.zeros_like(self.stall), self.stall)
        lag = (self.ref - self.s) * pressing
        cap = self.cap()
        # Bottomed: at the estimated full travel, or stalled again at the full cap -- the stop, wherever it truly is.
        bottomed = pressing & self.touched & ((self.s >= self.bottom - self.bottom_tol_m) | (stuck & ~now_touched))
        self.phase = torch.where(bottomed, torch.full_like(self.phase, HOLDING), self.phase)

        # PI along the axis, the integral kept only while it pushes the way the lag asks and never more than the cap.
        integral = self.integral + lag * dt
        integral = torch.where(integral * lag < 0.0, torch.zeros_like(integral), integral)
        integral = torch.minimum(integral.clamp(min=0.0), cap / self.ki)
        self.integral = integral * pressing
        push = (self.kp * lag + self.ki * self.integral).clamp(min=0.0)
        push = torch.minimum(push, cap)

        # Holding: a steady push for `hold_s`, then out.
        holding = active & (self.phase == HOLDING)
        self.held_s = torch.where(holding, self.held_s + dt, self.held_s)
        push = torch.where(holding, self.hold_n, push)
        release = holding & (self.held_s >= self.hold_s)
        self.phase = torch.where(release, torch.full_like(self.phase, RELEASING), self.phase)
        self.release_ref = torch.where(release, self.s, self.release_ref)
        releasing = active & (self.phase == RELEASING)
        self.release_ref = torch.where(releasing, (self.release_ref - self.release_speed * dt).clamp(min=0.0),
                                       self.release_ref)
        out = releasing & (self.s <= self.done_m)
        self.phase = torch.where(out, torch.full_like(self.phase, DONE), self.phase)
        push = torch.where(active & (self.phase >= RELEASING), torch.zeros_like(push), push)

        self.push = push * active
        self.force_cmd = self.push.unsqueeze(-1) * self.n
        depth = torch.where(self.phase <= HOLDING, self.s,
                            torch.where(self.phase == RELEASING, self.release_ref, torch.zeros_like(self.s)))
        goal = self.face + (depth - self.standoff).unsqueeze(-1) * self.n
        return goal, self.force_cmd


def tool_tensors(names, device) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """(tool index, length, radius) per name in `names`, as tensors."""
    index = torch.tensor([TOOL_NAMES.index(name) for name in names], device=device, dtype=torch.long)
    length = torch.tensor([TOOLS[name].length_m for name in names], device=device)
    radius = torch.tensor([TOOLS[name].radius_m for name in names], device=device)
    return index, length, radius


def tool_by_index(index: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """(length, radius) of the tools at `index` into `TOOL_NAMES`."""
    lengths = torch.tensor([TOOLS[name].length_m for name in TOOL_NAMES], device=index.device)
    radii = torch.tensor([TOOLS[name].radius_m for name in TOOL_NAMES], device=index.device)
    return lengths[index], radii[index]

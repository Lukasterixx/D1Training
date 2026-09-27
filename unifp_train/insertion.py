"""Inserting a held key into a keyed slot: the plant, and the task layer that drives the insertion.

Combiner boxes often have no handle: the door carries a key slot, and a removable handle is pushed into it and
turned (Lukas, 2026-09-26). This is the insertion, as the goal-commanded mechanism task (`mechanism.py`) is the
opening: an analytic plant stepped at the physics rate, so a training run can hold thousands of them.

**The key** is rigid in the hand: a flat blade whose end (the *tip*) sits `KEY_LENGTH_M` beyond the jaw centre
along the hand's approach axis, its long side along the jaw axis. A flat blade repeats every half turn, as a jaw
axis does, so the policy's roll command (`rewards.encode_roll`) is exactly the key's.

**The slot** is a rectangular channel into the door along its axis `n`, `2 half_u` by `2 half_w` across, the
long side along `u`; a 45° chamfer of width `chamfer` around the mouth; a detent `detent_depth` in that holds
until `detent_n` pushes past it and then clicks; and a seat at `depth`. The door's face is the plane through the
mouth normal to `n`.

**The contact**, a penalty model evaluated every physics step from where the key is and how it moves:

  * short of the face nothing touches;
  * a key that reaches the face outside the chamfered mouth -- too far off, or turned so its blade does not fit
    -- presses on the face: a spring back along `-n`, and Coulomb friction against sliding across it;
  * a key that reaches it inside the mouth -- straight on, or sliding across the face into it -- is *in*. From then
    on its blade is checked against the channel at the tip, where the chamfer ends and the channel is narrowest,
    and where it crosses the face (a tilted key is displaced by its tilt times the depth between),
    each side of each axis: interference with the chamfer pushes it back toward the centre and out along `-n`
    in equal parts (45°); interference with a wall pushes it toward the centre. Every wall and chamfer force
    loads friction along the axis, which is how a tilted key jams;
  * a blade rolled by `phi` is wider across the slot than one that is not (`blade_extent`), so a roll error eats
    the clearance and past a few degrees shuts the mouth;
  * the detent resists with up to `detent_n` over its last 1.5 mm, then lets go for good; the seat is a stop.

Everything is a force on the key's tip, in the world frame; the environment applies it there, so the hand feels
its moment too. The model is deliberately simple -- no rotational contact, one friction coefficient, no
compliance in the grip -- and the demo (`demos/insertion/`) inserts the same key in PhysX to say how far that
matters.

**The task layer** (`InsertionTaskLayer`) is what a robot would run, knowing the slot only from an estimate:
the goal to the estimated mouth, then a force law along the estimated axis on how far the tip lags a reference
depth (the mechanism task's PI, F-106), capped low while the key is still on the face and a spiral search across
the face when it stalls there, then the full cap once the key is in. Only the tip's position is read, as the arm's
kinematics give it.

Plain torch, no Isaac imports (`tests/test_insertion.py`).
"""
from __future__ import annotations

import math

import torch

#: The key: the tip's distance beyond the jaw centre along the approach, and the blade's half sizes (long, short).
KEY_LENGTH_M = 0.07
KEY_HALF_U_M = 0.005
KEY_HALF_W_M = 0.0015
#: Penalty contact: stiffness and damping of every face, wall and chamfer, and the seat. Stiff enough that pushing a
#: key onto the face does not sink it millimetres in -- at 8,000 N/m a 30 N push read as 4 mm "past the face", which
#: the task layer took for an insertion (the fourth run, 2026-09-26) -- and still stable at the 5 ms physics step.
CONTACT_K_N_PER_M = 16000.0
CONTACT_C_N_S_PER_M = 50.0
#: Below this sliding speed, m/s, Coulomb friction is ramped rather than switched. A sharp switch against an explicit
#: penalty contact at the 5 ms physics step chatters: the sign flips every step and the key never settles.
FRICTION_V_EPS = 0.02
#: The detent acts over this much travel before its depth.
DETENT_TRAVEL_M = 0.0015
#: A key backed out this far past the face is out again.
EXIT_M = 0.002
#: The door is a panel this far across either way from the mouth and this thick: a key pushed this deep into its
#: face outside the slot has gone through it (`pierced`, which the environment treats as damage). No penalty force is
#: allowed past this interference either, so a hand that starts somewhere unphysical is not flung by thousands of
#: newtons of explicit spring (the second training run, 2026-09-26, collapsed on exactly that).
PANEL_HALF_M = 0.15
FACE_DEPTH_M = 0.01
MAX_OVER_M = 0.01


def unit(v: torch.Tensor) -> torch.Tensor:
    return v / v.norm(dim=-1, keepdim=True).clamp(min=1e-9)


def roll_frame(axis: torch.Tensor, roll: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """(u, w): the direction a blade rolled by `roll` about `axis` lies along, and `axis x u`.

    The roll is measured as `rewards.tool_roll` measures the jaws: from the horizontal direction perpendicular to
    the axis, so 0 is a level blade and pi/2 an upright one. `u = cos(roll) r + sin(roll) (axis x r)` with
    `r = up x axis`, normalised.
    """
    up = torch.zeros_like(axis)
    up[:, 2] = 1.0
    reference = unit(torch.cross(up, axis, dim=-1))
    second = torch.cross(axis, reference, dim=-1)
    r = roll.unsqueeze(-1)
    u = torch.cos(r) * reference + torch.sin(r) * second
    return u, torch.cross(axis, u, dim=-1)


def blade_extent(half_u: float, half_w: float, phi: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """Half extents along the slot's (u, w) of a blade rolled `phi` from the slot's long axis."""
    c, s = torch.cos(phi).abs(), torch.sin(phi).abs()
    return half_u * c + half_w * s, half_u * s + half_w * c


def wrap_half_turn(angle: torch.Tensor) -> torch.Tensor:
    return torch.remainder(angle + math.pi / 2, math.pi) - math.pi / 2


class Slot:
    """One keyed slot per environment, and the key's contact with it."""

    def __init__(self, num_envs: int, device, *, key_half_u: float = KEY_HALF_U_M, key_half_w: float = KEY_HALF_W_M,
                 k: float = CONTACT_K_N_PER_M, c: float = CONTACT_C_N_S_PER_M) -> None:
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.key_half_u, self.key_half_w, self.k, self.c = key_half_u, key_half_w, k, c
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        zeros3 = lambda: torch.zeros(num_envs, 3, device=self.device)
        flags = lambda: torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.mouth, self.n, self.u, self.w = zeros3(), zeros3(), zeros3(), zeros3()
        self.half_u, self.half_w, self.chamfer, self.depth = zeros(), zeros(), zeros(), zeros()
        self.detent_depth, self.detent_n, self.mu = zeros(), zeros(), zeros()
        self.active = flags()
        #: State: the key is in the mouth; the detent has clicked; how deep the tip is past the face.
        self.entered = flags()
        self.clicked = flags()
        self.pierced = flags()
        self.tip_depth = zeros()
        self.prev_depth = zeros()
        #: As of the last `step()`: force on the key's tip (world), and its parts, N.
        self.force_on_tip = zeros3()
        self.axial_n = zeros()      # along -n (pushing the key back out), positive
        self.lateral_n = zeros()    # across the axis, magnitude
        self.normal_sum_n = zeros() # every contact's normal force, summed (what loads friction)

    def place(self, env_ids: torch.Tensor, *, mouth: torch.Tensor, axis: torch.Tensor, roll: torch.Tensor,
              half_u: torch.Tensor, half_w: torch.Tensor, chamfer: torch.Tensor, depth: torch.Tensor,
              detent_depth: torch.Tensor, detent_n: torch.Tensor, mu: torch.Tensor) -> None:
        n = unit(axis)
        u, w = roll_frame(n, roll)
        self.mouth[env_ids], self.n[env_ids], self.u[env_ids], self.w[env_ids] = mouth, n, u, w
        self.half_u[env_ids], self.half_w[env_ids] = half_u, half_w
        self.chamfer[env_ids], self.depth[env_ids] = chamfer, depth
        self.detent_depth[env_ids], self.detent_n[env_ids], self.mu[env_ids] = detent_depth, detent_n, mu
        self.active[env_ids] = True
        self.entered[env_ids] = False
        self.clicked[env_ids] = False
        self.pierced[env_ids] = False
        self.tip_depth[env_ids] = -1.0
        self.prev_depth[env_ids] = -1.0

    def reset(self, env_ids: torch.Tensor) -> None:
        self.active[env_ids] = False
        self.entered[env_ids] = False
        self.clicked[env_ids] = False
        self.pierced[env_ids] = False
        self.force_on_tip[env_ids] = 0.0
        self.axial_n[env_ids] = 0.0
        self.lateral_n[env_ids] = 0.0
        self.normal_sum_n[env_ids] = 0.0

    def seated(self, tolerance_m: float = 0.001) -> torch.Tensor:
        return self.active & self.entered & (self.tip_depth >= self.depth - tolerance_m)

    def step(self, tip: torch.Tensor, tip_vel: torch.Tensor, key_axis: torch.Tensor, key_long: torch.Tensor,
             dt: float) -> torch.Tensor:
        """The force on the key's tip (world, (N, 3)) with the key at `tip`, pointing `key_axis`, its blade along
        `key_long`; `tip_vel` is the tip's world velocity. Updates the entered/clicked state."""
        r = self._contact(tip, tip_vel, key_axis, key_long)
        self.entered, self.pierced, self.clicked = r["entered"], r["pierced"], r["clicked"]
        active, d = self.active, r["d"]
        self.prev_depth = torch.where(active, d, self.prev_depth)
        self.tip_depth = torch.where(active, d, self.tip_depth)
        self.force_on_tip = r["force"]
        self.axial_n = torch.where(active, r["axial_n"], torch.zeros_like(d))
        self.lateral_n = torch.where(active, r["lateral_n"], torch.zeros_like(d))
        self.normal_sum_n = torch.where(active, r["normal_sum"], torch.zeros_like(d))
        return self.force_on_tip

    def probe(self, tip: torch.Tensor, tip_vel: torch.Tensor, key_axis: torch.Tensor,
              key_long: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """(the force on the tip, the summed normal force) `step` would give, changing nothing: for a compliant grip
        to settle against the contact before the step is taken (`CompliantGrip`)."""
        r = self._contact(tip, tip_vel, key_axis, key_long)
        return r["force"], torch.where(self.active, r["normal_sum"], torch.zeros_like(r["d"]))

    def _contact(self, tip: torch.Tensor, tip_vel: torch.Tensor, key_axis: torch.Tensor,
                 key_long: torch.Tensor) -> dict[str, torch.Tensor]:
        """Every contact force with the key at `tip`, and the entered/pierced/clicked state it leaves: pure."""
        q = tip - self.mouth
        d = (q * self.n).sum(-1)
        e_u, e_w = (q * self.u).sum(-1), (q * self.w).sum(-1)
        v_n = (tip_vel * self.n).sum(-1)
        v_lat = tip_vel - v_n.unsqueeze(-1) * self.n
        # Tilt of the key off the axis, per slot axis (the tip leads the face crossing by this per metre of depth).
        a = unit(key_axis)
        cos_tilt = (a * self.n).sum(-1).clamp(min=0.2)
        t_u, t_w = (a * self.u).sum(-1) / cos_tilt, (a * self.w).sum(-1) / cos_tilt
        # Roll of the blade from the slot's long axis, as an axis (half-turn periodic).
        phi = wrap_half_turn(torch.atan2((key_long * self.w).sum(-1), (key_long * self.u).sum(-1)))
        ext_u, ext_w = blade_extent(self.key_half_u, self.key_half_w, phi)
        clear_u, clear_w = self.half_u - ext_u, self.half_w - ext_w

        # Entering: at the face (not pressed through it past the chamfer's depth), inside the chamfered mouth, with
        # the blade able to fit at all -- whether it arrives there straight or slides across the face into it.
        at_face = self.active & ~self.entered & (d >= 0.0) & (d <= self.chamfer)
        fits = (clear_u > 0.0) & (clear_w > 0.0) & (e_u.abs() < clear_u + self.chamfer) & (e_w.abs() < clear_w + self.chamfer)
        entered = (self.entered | (at_face & fits)) & ~(d < -EXIT_M)

        force = torch.zeros_like(tip)
        normal_sum = torch.zeros_like(d)
        # On the face -- the panel's front, not an infinite plane -- pressed back, friction across.
        on_panel = (e_u.abs() < PANEL_HALF_M) & (e_w.abs() < PANEL_HALF_M)
        on_face = self.active & ~entered & (d > 0.0) & on_panel
        pierced = self.pierced | (on_face & (d >= FACE_DEPTH_M))
        press = (self.k * d.clamp(max=FACE_DEPTH_M) + self.c * v_n.clamp(min=0.0)) * on_face
        force = force - press.unsqueeze(-1) * self.n
        normal_sum = normal_sum + press

        # In the slot: each axis, each side, at the tip and where the blade crosses the face.
        inside = self.active & entered
        axial = torch.zeros_like(d)
        lateral = torch.zeros_like(tip)
        for e, t, clear, axis_dir in ((e_u, t_u, clear_u, self.u), (e_w, t_w, clear_w, self.w)):
            # The tip, where the chamfer ends and the channel is narrowest, and the face.
            for z in (d.clamp(min=0.0), torch.minimum(d.clamp(min=0.0), self.chamfer), torch.zeros_like(d)):
                offset = e - t * (d - z)                                        # the blade's centre at depth z
                opening = clear + (self.chamfer - z).clamp(min=0.0)             # the chamfer widens the mouth
                over = (offset.abs() - opening).clamp(min=0.0, max=MAX_OVER_M) * inside
                push = self.k * over
                in_chamfer = z < self.chamfer
                # Chamfer: a 45° face, half back along -n and half toward the centre; wall: toward the centre.
                lat = torch.where(in_chamfer, push * math.sqrt(0.5), push)
                ax = torch.where(in_chamfer, push * math.sqrt(0.5), torch.zeros_like(push))
                lateral = lateral - (torch.sign(offset) * lat).unsqueeze(-1) * axis_dir
                axial = axial + ax
                normal_sum = normal_sum + push
        # Friction along the axis from every in-slot contact, against the tip's axial motion.
        friction = self.mu * (normal_sum - press) * torch.tanh(v_n / FRICTION_V_EPS)
        axial = axial + friction * inside
        # The detent's last 1.5 mm, until it clicks; the seat.
        into_detent = inside & ~self.clicked & (d > self.detent_depth - DETENT_TRAVEL_M) & (self.detent_n > 0.0)
        detent = torch.where(into_detent, self.detent_n * ((d - (self.detent_depth - DETENT_TRAVEL_M)) / DETENT_TRAVEL_M).clamp(0.0, 1.0),
                             torch.zeros_like(d))
        clicked = self.clicked | (inside & (d >= self.detent_depth))
        seat = (self.k * (d - self.depth).clamp(min=0.0, max=MAX_OVER_M) + self.c * v_n.clamp(min=0.0) * (d > self.depth)) * inside
        axial = axial + detent + seat
        force = force + lateral - axial.unsqueeze(-1) * self.n
        # Sliding on the face: Coulomb against the lateral motion.
        speed = v_lat.norm(dim=-1, keepdim=True)
        face_friction = -(self.mu * press).unsqueeze(-1) * v_lat / speed.clamp(min=1e-9) * torch.tanh(speed / FRICTION_V_EPS)
        force = force + face_friction
        return {"d": d, "entered": entered, "pierced": pierced, "clicked": clicked,
                "force": torch.where(self.active.unsqueeze(-1), force, torch.zeros_like(force)),
                "axial_n": (press + axial).clamp(min=0.0), "lateral_n": (lateral + face_friction).norm(dim=-1),
                "normal_sum": normal_sum}


class InsertionTaskLayer:
    """The insertion as a robot would run it, from an estimate of the slot and the tip's measured position.

    Per environment: the estimated mouth, axis and roll; a reference depth that runs from the standoff to the
    seat at a set speed; a PI law along the axis on how far the tip lags it (capped at `search_n` until the key is
    in, at `max_n` after); and a spiral across the face, grown only while the tip is stalled on it. The goal it
    returns is for the tip -- along the axis where the tip is (the law carries the push), across the axis where
    the estimate and the search put it -- and the force is along the estimated axis.
    """

    def __init__(self, num_envs: int, device, *, kp: float = 400.0, ki: float = 1000.0, bleed_s: float = 1.0,
                 stall_s: float = 0.3, spiral_pitch_m: float = 0.0015, spiral_speed_m_s: float = 0.01,
                 spiral_max_m: float = 0.006, entered_m: float = 0.004, arrived_m: float = 0.001,
                 lead_m: float = 0.004, stalled_m_s: float = 0.01, comply_s: float = 0.15,
                 speed_filter_s: float = 0.1, home_m: float = 0.001, hold_n: float = 5.0) -> None:
        self.device = torch.device(device)
        self.kp, self.ki, self.bleed_s, self.stall_s = kp, ki, bleed_s, stall_s
        self.spiral_pitch, self.spiral_speed, self.spiral_max = spiral_pitch_m, spiral_speed_m_s, spiral_max_m
        self.entered_m, self.arrived_m, self.lead_m, self.stalled_m_s = entered_m, arrived_m, lead_m, stalled_m_s
        self.comply_s, self.speed_filter_s = comply_s, speed_filter_s
        self.home_m, self.hold_n = home_m, hold_n
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        zeros3 = lambda: torch.zeros(num_envs, 3, device=self.device)
        flags = lambda: torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.mouth, self.n, self.u, self.w = zeros3(), zeros3(), zeros3(), zeros3()
        self.standoff, self.depth, self.speed = zeros(), zeros(), zeros()
        self.search_n, self.max_n = zeros(), zeros()
        self.active = flags()
        #: State. `s` is the tip's depth past its start (the standoff short of the estimated face).
        self.s, self.ref, self.integral, self.stall, self.theta = zeros(), zeros(), zeros(), zeros(), zeros()
        self.v = zeros()
        self.offset = zeros3()
        self.inserted = flags()
        #: The key has reached the seat (its tip within `home_m` of the travel, once in): from then on the goal holds it
        #: there and the push is a light `hold_n`. Without it the goal along the axis only follows the tip, and a seated
        #: key the hand lets drift back out is banged in again (11 of 16 damaged keys had seated first, 2026-09-27).
        self.home = flags()
        #: Stalled on the face at least once: the key has touched, so the goal may give to it across the axis.
        self.touched = flags()
        self.force_cmd = zeros3()

    def start(self, env_ids: torch.Tensor, *, mouth: torch.Tensor, axis: torch.Tensor, roll: torch.Tensor,
              standoff: torch.Tensor, depth: torch.Tensor, speed: torch.Tensor, search_n: torch.Tensor,
              max_n: torch.Tensor) -> None:
        n = unit(axis)
        u, w = roll_frame(n, roll)
        self.mouth[env_ids], self.n[env_ids], self.u[env_ids], self.w[env_ids] = mouth, n, u, w
        self.standoff[env_ids], self.depth[env_ids], self.speed[env_ids] = standoff, depth, speed
        self.search_n[env_ids], self.max_n[env_ids] = search_n, max_n
        self.active[env_ids] = True
        for buf in (self.s, self.ref, self.integral, self.stall, self.theta, self.v):
            buf[env_ids] = 0.0
        self.offset[env_ids] = 0.0
        self.inserted[env_ids] = False
        self.home[env_ids] = False
        self.touched[env_ids] = False
        self.force_cmd[env_ids] = 0.0

    def reset(self, env_ids: torch.Tensor) -> None:
        self.active[env_ids] = False
        self.force_cmd[env_ids] = 0.0
        self.integral[env_ids] = 0.0

    @property
    def travel(self) -> torch.Tensor:
        return self.standoff + self.depth

    def step(self, tip: torch.Tensor, dt: float) -> tuple[torch.Tensor, torch.Tensor]:
        """(the tip's goal, the force command), world frame, from the tip's measured position."""
        active = self.active
        s = ((tip - self.mouth) * self.n).sum(-1) + self.standoff
        # The tip's speed along the axis, low-passed: contact chatters, and a stall is read off this.
        raw = (s - self.s) / dt
        self.v = torch.where(active, self.v + (raw - self.v) * min(1.0, dt / self.speed_filter_s), self.v)
        self.s = torch.where(active, s, self.s)
        # The reference runs to the seat at its speed whatever the tip does, so a hand short of the door is behind and
        # knows it. When the tip is first clearly past the estimated face the key is in: the reference is brought back
        # to `lead_m` ahead of it and the integral cleared, so what built up on the face does not ram the key home.
        now_in = active & ~self.inserted & (self.s > self.standoff + self.entered_m)
        self.inserted |= now_in
        ref = torch.minimum(self.ref + self.speed * dt, self.travel)
        ref = torch.where(now_in, torch.minimum(ref, self.s + self.lead_m), ref)
        self.ref = torch.where(active, ref, self.ref)
        self.integral = torch.where(now_in, torch.zeros_like(self.integral), self.integral)
        lag = (self.ref - self.s) * active
        # Stalled: not in, asked to go further, and not moving along the axis -- the tip is on the door's face.
        on_face = active & ~self.inserted & (lag > 0.5 * self.lead_m) & (self.v.abs() < self.stalled_m_s)
        self.stall = torch.where(on_face, self.stall + dt, torch.zeros_like(self.stall))
        searching = self.stall > self.stall_s
        self.touched |= searching
        self.theta = torch.where(searching, self.theta + self.spiral_speed * dt / (self.spiral_radius() + 1e-3), self.theta)
        radius = self.spiral_radius()
        spiral = radius.unsqueeze(-1) * (torch.cos(self.theta).unsqueeze(-1) * self.u
                                         + torch.sin(self.theta).unsqueeze(-1) * self.w)
        # Across the axis: the spiral while searching; otherwise, once the key has touched, the goal gives to where
        # the tip is (a chamfer centring the key can only do so if the hand yields). Never before a touch, where
        # following the tip would integrate the policy's own tracking error.
        lateral = (tip - self.mouth) - ((tip - self.mouth) * self.n).sum(-1, keepdim=True) * self.n
        comply = active & self.touched & ~searching
        followed = self.offset + (lateral - self.offset) * min(1.0, dt / self.comply_s)
        self.offset = torch.where(comply.unsqueeze(-1), followed, self.offset)
        self.offset = torch.where((active & searching).unsqueeze(-1), spiral, self.offset)
        # PI along the axis, the integral kept only while it pushes the way the lag asks, bled once arrived, and --
        # before the key is in -- never more than the face's cap on its own, so it cannot wind up while the key waits.
        cap = torch.where(self.inserted, self.max_n, self.search_n)
        integral = self.integral + lag * dt
        integral = torch.where(integral * lag < 0.0, torch.zeros_like(integral), integral)
        arrived = lag.abs() < self.arrived_m
        integral = torch.where(arrived, integral * math.exp(-dt / self.bleed_s), integral)
        integral = torch.where(self.inserted, integral, integral.clamp(-cap / self.ki, cap / self.ki))
        self.integral = integral * active
        push = (self.kp * lag + self.ki * self.integral).clamp(min=-cap, max=cap)
        # Home: hold the key at the seat with a light push, and stop driving it.
        self.home |= active & self.inserted & (self.s >= self.travel - self.home_m)
        push = torch.where(self.home, torch.minimum(torch.full_like(push, self.hold_n), cap), push)
        self.force_cmd = (push * active).unsqueeze(-1) * self.n
        depth = torch.where(self.home, self.travel, self.s)
        goal = self.mouth + ((depth - self.standoff).unsqueeze(-1)) * self.n + self.offset
        return goal, self.force_cmd

    def spiral_radius(self) -> torch.Tensor:
        return (self.spiral_pitch * self.theta / (2 * math.pi)).clamp(max=self.spiral_max)


#: The compliant grip (`CompliantGrip`, from 2026-09-27): rotational stiffness and damping of the key about the jaw
#: centre, across its axis, and how far it may tilt. At 2 N·m/rad a force of 2.5 N at the tip, 70 mm out, tilts the
#: key 5°. The damping is light (a 10 ms time constant against the spring): a 20 g key in a soft holder is nearly
#: quasi-static at the 10 Hz the policies jitter at, and a heavy damper would itself push the tip into the walls.
GRIP_STIFFNESS_NM_PER_RAD = 2.0
GRIP_DAMPING_NMS_PER_RAD = 0.02
GRIP_MAX_RAD = math.radians(10.0)
#: Implicit relaxations of the grip against the probed contact per physics step, before the step is committed.
GRIP_SETTLE_ITERATIONS = 2


class CompliantGrip:
    """The key held so it can tilt in the hand: a rotational spring and damper about the jaw centre, on the two axes
    across the key, as a remote-centre-compliance holder or a key in rubber-lined jaws would hold it.

    A rigid key must arrive within a few degrees of the slot's axis (a 1 mm clearance over 18 mm of channel admits
    about 3-6°), which the whole-body policy does not deliver and the estimate alone may miss by 3°. With the grip
    compliant, the chamfer's and the walls' forces at the tip turn the key into line instead of wedging it. Its roll
    about its own axis stays rigid, as a blade held flat between jaws would.

    `tilt` is (about the hand's x, about its y), rad: `Ry(tilt_y) Rx(tilt_x)` takes the hand's approach (+z) to the
    key's axis and its jaw axis (+y) to the blade's long direction. Plain torch; stepped at the physics rate.
    """

    def __init__(self, num_envs: int, device, *, stiffness: float = GRIP_STIFFNESS_NM_PER_RAD,
                 damping: float = GRIP_DAMPING_NMS_PER_RAD, max_rad: float = GRIP_MAX_RAD) -> None:
        self.stiffness, self.damping, self.max = stiffness, damping, max_rad
        self.tilt = torch.zeros(num_envs, 2, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        self.tilt[env_ids] = 0.0

    def axes_local(self) -> tuple[torch.Tensor, torch.Tensor]:
        """(the key's axis, the blade's long direction), unit, in the hand's frame."""
        cx, sx = torch.cos(self.tilt[:, 0]), torch.sin(self.tilt[:, 0])
        cy, sy = torch.cos(self.tilt[:, 1]), torch.sin(self.tilt[:, 1])
        axis = torch.stack((cx * sy, -sx, cx * cy), dim=-1)
        long = torch.stack((sx * sy, cx, sx * cy), dim=-1)
        return axis, long

    def step(self, force_local: torch.Tensor, length: float, dt: float,
             contact_stiffness: torch.Tensor | float = 0.0) -> None:
        """Tilt under `force_local` (N, the hand's frame) acting at the tip, `length` out along the key.

        Backward Euler, linearised: the contact's own rotational stiffness about the grip (`contact_stiffness`,
        N·m/rad, e.g. the penalty stiffness times the length squared while the key touches) is taken implicitly
        with the spring's, so the step is stable however light the damping -- an explicit step against an
        80 N·m/rad contact would need the damping forty times heavier than a real holder's.
        """
        axis, _ = self.axes_local()
        torque = torch.cross(length * axis, force_local, dim=-1)
        stiff = self.stiffness + (contact_stiffness.unsqueeze(-1) if torch.is_tensor(contact_stiffness)
                                  else contact_stiffness)
        tilt = self.tilt + dt * (torque[:, :2] - self.stiffness * self.tilt) / (self.damping + dt * stiff)
        self.tilt = tilt.clamp(-self.max, self.max)

    def settle(self, probe, length: float, dt: float, iterations: int = GRIP_SETTLE_ITERATIONS) -> None:
        """Relax the tilt over `dt` against the contact the hand is now in, before that contact is committed.

        `probe(axis_local, long_local) -> (force_local, touching)` evaluates the contact with the key at a trial
        tilt (`Slot.probe`, in the hand's frame). A hand moving 0.3 m/s crosses the whole clearance in one 5 ms
        physics step, so a tilt that only answered the previous step's contact would meet every wall late, and the
        key would be banged as hard as a rigid one.
        """
        stiffness = CONTACT_K_N_PER_M * length ** 2
        for _ in range(iterations):
            axis, long = self.axes_local()
            force, touching = probe(axis, long)
            self.step(force, length, dt / iterations, touching.float() * stiffness)


class PoseCorrection:
    """The task layer's outer loops on what it measures: the key's tip, the hand's approach elevation, the blade's roll.

    The whole-body policy never observes its own hand (F-118): it turns a goal into joint angles through the arm it was
    trained on, and arrives centimetres and degrees off -- steadily, for a steady goal. These loops integrate the
    measured miss into the commands until the key, not the command, is where the task layer wants it:

      * the jaw goal is placed so the *tip* lands on its goal given the hand's measured approach (the key is
        `key_length` beyond the jaw along it), plus `offset`, the integral of the tip's miss;
      * the approach's commanded elevation is the wanted one plus `elevation`, the integral of its miss;
      * the commanded roll is the wanted one plus `roll`, the integral of the blade's roll miss (half-turn periodic).

    Each integral is clamped, and integrates only where its mask says (the environment's: while the hand holds the
    pre-insertion pose, and during the insertion until contact makes the miss the slot's business). On a robot the
    tip and the hand's attitude would come from the arm's kinematics or a wrist camera; here, from the simulator.
    """

    def __init__(self, num_envs: int, device, *, tip_gain: float, tip_max_m: float, elevation_gain: float,
                 elevation_max_rad: float, roll_gain: float, roll_max_rad: float) -> None:
        self.tip_gain, self.tip_max = tip_gain, tip_max_m
        self.elevation_gain, self.elevation_max = elevation_gain, elevation_max_rad
        self.roll_gain, self.roll_max = roll_gain, roll_max_rad
        self.offset = torch.zeros(num_envs, 3, device=device)
        self.elevation = torch.zeros(num_envs, device=device)
        self.roll = torch.zeros(num_envs, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        self.offset[env_ids] = 0.0
        self.elevation[env_ids] = 0.0
        self.roll[env_ids] = 0.0

    def jaw_for_tip(self, tip_goal: torch.Tensor, approach: torch.Tensor, key_length: float) -> torch.Tensor:
        """Where to send the jaw centre for the tip to reach `tip_goal`, the hand pointing along `approach`."""
        return tip_goal - key_length * approach + self.offset

    def update_tip(self, mask: torch.Tensor, tip_goal: torch.Tensor, tip: torch.Tensor, dt: float,
                   across: torch.Tensor | None = None) -> None:
        """Integrate the tip's miss where `mask`; only its part across the unit `across` axis, if given."""
        if self.tip_gain <= 0.0:
            return
        miss = tip_goal - tip
        if across is not None:
            miss = miss - (miss * across).sum(-1, keepdim=True) * across
        offset = self.offset + self.tip_gain * dt * miss
        offset = offset * (self.tip_max / offset.norm(dim=-1, keepdim=True).clamp(min=1e-9)).clamp(max=1.0)
        self.offset = torch.where(mask.unsqueeze(-1), offset, self.offset)

    def update_attitude(self, mask: torch.Tensor, elevation_want: torch.Tensor, elevation_now: torch.Tensor,
                        roll_want: torch.Tensor, roll_now: torch.Tensor, dt: float) -> None:
        """Integrate the approach's elevation miss and the blade's roll miss where `mask`, radians."""
        if self.elevation_gain > 0.0:
            elevation = (self.elevation + self.elevation_gain * dt * (elevation_want - elevation_now)).clamp(
                -self.elevation_max, self.elevation_max)
            self.elevation = torch.where(mask, elevation, self.elevation)
        if self.roll_gain > 0.0:
            roll = (self.roll + self.roll_gain * dt * wrap_half_turn(roll_want - roll_now)).clamp(
                -self.roll_max, self.roll_max)
            self.roll = torch.where(mask, roll, self.roll)

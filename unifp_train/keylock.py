"""Lukas's cam lock and the PLA lever key in the D1's jaws, for the arm's insertion (`keyinsert_env.py`). Plain torch.

Lukas, 2026-11-13: "train the key insertion to be force aware so it can help by feeling where the lock is. model the
lock off the lever's specific lock shape and assume the lever is plastic and the lock is metal."

**The lock** is the box's cam lock as Lukas measured it (`demos/combiner_key/robot/insert_plan.py`, 2026-10-21, callipers):
a chrome cylinder whose face stands `LOCK_FACE_OFF_DOOR_M` (4.5 mm) proud of the door, its keyway the lever's own
profile -- the 10 mm round shaft and its two wards, 15 mm across and 3.5 mm thick, the wards 45 degrees off the bar
(v4, `geometry.LEVER_V4.ward_angle_deg` 135) -- `KEYWAY_DEPTH_M` (8.7 mm) deep, with "under 0.5 mm of play" (drawn here
0.1-0.25 mm a side), behind a small 45-degree break on the keyway's edge (drawn 0.2-0.8 mm; not measured). The face is a
plane (the 27.5 mm bezel round the cylinder is flush enough at the search's 5 mm). The keyway's floor stops the shaft,
which is longer than the keyway is deep: seated, the key's back face is 5.8 mm off the door.

**The key** is v4's shaft and wards (`demos/combiner_key/geometry`), printed PLA, rigid in the jaws: the dog's grip, the
shaft on the jaws' axis (Link6's z), roll +1 (`leg_cfg.KEYHOLD_GRIP_ROLL`), its back face at Link6 z 0.1266 (the jaw
centre is at 0.1051), its tip `KEY_LENGTH_M` further on. The printed tip's edges are broken by a drawn 0.3-1.0 mm.
It can slide back in the jaws: past a drawn axial grip force the jaws slip (`slip_m`), as the dog's grip on the bar
would; the jaws themselves are not modelled.

**The contact** is a penalty model at `POINTS` sample points on the key (its tip's face, the edges round it, its sides
at three depths), each tested against the lock's metal: in front of the face nothing; behind it, inside the keyway (the
shaft's circle and the wards' slot, grown by the edge break near the face) nothing until the floor; elsewhere the point
is pushed out the nearest way -- back through the face, out through the edge break at 45 degrees, or sideways into the
keyway. Each point carries a spring and a damper along that normal and regularised Coulomb friction across it: PLA on
chromed metal, mu 0.2-0.45 (dry; drawn, not measured). Summed, the force and its moment about the jaw centre act on
Link6. The stiffness is the 5 ms physics step's, not the materials': a printed shaft on a steel face is far stiffer
(`CONTACT_K_N_PER_M`'s note). The cylinder does not turn.

**The task layer** (`InsertLayer`) is what the dog would run on the arm's reported joints, the dog program's insertion
(`node.py --task insert`) made continuous: in along the estimated axis until the hand falls `CONTACT_LAG_M` behind its
reference (the face, felt by the lag), then an Archimedean spiral across the face with the wrist rolled +-`WIGGLE_DEG`,
the key pressed on; in when it advances `ENTER_M` (or is held sideways: then it is already in the keyway), on to seated,
held. For the force-aware policy the press is a commanded force and the goal is set back from the reference by the
force over the reward's compliance, so both variants run the same reference (`InsertLayer.goal`).
"""
from __future__ import annotations

import math

import torch

# --- the key (v4, PLA), in its own frame: u along the bar, t up its top side, h from the back face toward the tip ----

SHAFT_R_M = 0.005
WARD_HALF_LEN_M = 0.0075
WARD_HALF_WIDTH_M = 0.00175
KEY_LENGTH_M = 0.010
#: The wards' direction about the shaft, atan2(t, u) (v4: 45 degrees off the bar).
WARD_ANGLE_RAD = math.radians(135.0)
#: The key's back face and the jaw centre in Link6 (`geometry.KEY_ORIGIN_LINK6`, `arm_kin.JAW_CENTRE_LINK6_M`).
KEY_BACK_LINK6_Z_M = 0.1266
JAW_LINK6_Z_M = 0.1051
#: From the jaw centre to the key's tip along the shaft, as held.
TIP_FROM_JAW_M = KEY_BACK_LINK6_Z_M + KEY_LENGTH_M - JAW_LINK6_Z_M

# --- the lock (metal), measured ---------------------------------------------------------------------------------------

LOCK_FACE_OFF_DOOR_M = 0.0045
KEYWAY_DEPTH_M = 0.0087
#: Drawn per lock (training ranges): the keyway's clearance a side, its edge break, the key tip's edge break, friction.
PLAY_M = (0.10e-3, 0.25e-3)
LOCK_CHAMFER_M = (0.2e-3, 0.8e-3)
KEY_CHAMFER_M = (0.3e-3, 1.0e-3)
MU = (0.20, 0.45)
#: The jaws let the key slide back past this axial force (N): the D1's grip on a 15 mm PLA bar, not measured.
GRIP_SLIP_N = (20.0, 35.0)
#: How fast it slides per newton past that (m/s/N), and how far it may go before the key counts as lost.
SLIP_RATE_M_S_N = 0.0005
SLIP_LOST_M = 0.012

#: The whole tip face's stiffness pressed flat on the face (N/m), shared among the points on the tip's face; every other
#: point gets a share as large. Explicit penalty contact at the 5 ms physics step against the D1's light wrist is stable
#: to about this (the slot plant's 16,000, `insertion.CONTACT_K_N_PER_M`); a PLA tip on steel is several orders stiffer,
#: so the simulated key sinks ~1 mm into the face at 12 N where the real one would not.
CONTACT_K_N_PER_M = 12000.0
CONTACT_C_N_S_PER_M = 40.0
#: ... shared as though `POINTS_SHARE` points carried it: the tip's 35 face points pressed flat give 1.75 times the
#: stiffness (21,000 N/m), and a key resting on its four ward corners is not as soft as 4 / 35 of it.
POINTS_SHARE = 20
#: No point pushes harder than this (N): a hand swinging into the face at speed takes points millimetres into the metal
#: in one 5 ms step, and uncapped round 1's position-only run met 19 kN (2026-11-13). 119 points cap the key at ~240 N.
POINT_MAX_N = 2.0
#: Below this sliding speed (m/s) friction is ramped, not switched.
FRICTION_V_REG_M_S = 0.004
#: In the keyway (scoring and reward): the shaft's axis within the play plus this of the keyway's, and the tip deeper than
#: this; seated: within this of the floor.
INSIDE_GIVE_M = 0.0005
INSIDE_DEPTH_M = 0.002
SEATED_WITHIN_M = 0.0008
#: The contact force past which the PLA key counts as damaged (N): its 3.5 mm wards bent at the root, or the shaft's
#: layers parted. An estimate from PLA's ~25 MPa across layers, not a test.
DAMAGE_N = 50.0
#: The press the dog should stay under (N): the reward's line.
PRESS_LIMIT_N = 20.0


def key_points(key_chamfer: torch.Tensor) -> tuple[torch.Tensor, int]:
    """The sample points (N, P, 3) in Link6's frame for keys whose tip edges are broken by `key_chamfer` (N,), and how
    many of them lie on the tip's face (the first ones). Link6: x along the bar (u), y = -t, z along the shaft."""
    n, device = key_chamfer.shape[0], key_chamfer.device
    c = key_chamfer.unsqueeze(-1)                                      # (N, 1)
    rows_u, rows_t, rows_h = [], [], []

    def add(u, t, h):
        rows_u.append(u)
        rows_t.append(t)
        rows_h.append(h)

    tip = torch.full((n, 1), KEY_LENGTH_M, device=device)
    # The tip's face: the shaft's rim inside the break, the centre and a ring, the wards' front faces inside the break.
    ang = torch.arange(16, device=device) * (2 * math.pi / 16)
    add((SHAFT_R_M - c) * torch.cos(ang), (SHAFT_R_M - c) * torch.sin(ang), tip.expand(n, 16))
    ring = torch.arange(6, device=device) * (2 * math.pi / 6)
    add(torch.cat((torch.zeros(n, 1, device=device), 0.0025 * torch.cos(ring).expand(n, 6)), 1),
        torch.cat((torch.zeros(n, 1, device=device), 0.0025 * torch.sin(ring).expand(n, 6)), 1), tip.expand(n, 7))
    cw, sw = math.cos(WARD_ANGLE_RAD), math.sin(WARD_ANGLE_RAD)
    along_front = []
    for end in (1.0, -1.0):
        for a in (0.0055, 0.0065, None):
            for b in (1.0, -1.0):
                along_front.append((end, a, b))
    fu, ft = [], []
    for end, a, b in along_front:
        a_m = (WARD_HALF_LEN_M - c) if a is None else torch.full((n, 1), a, device=device)
        b_m = b * (WARD_HALF_WIDTH_M - c.clamp(max=0.0008))
        fu.append(end * a_m * cw - b_m * sw)
        ft.append(end * a_m * sw + b_m * cw)
    add(torch.cat(fu, 1), torch.cat(ft, 1), tip.expand(n, len(along_front)))
    face = sum(x.shape[1] for x in rows_u)
    # The break's outer edge, one break behind the tip: the shaft's full circle and the wards' corners.
    add(SHAFT_R_M * torch.cos(ang).expand(n, 16), SHAFT_R_M * torch.sin(ang).expand(n, 16), (tip - c).expand(n, 16))
    corners = [(e, a, b) for e in (1.0, -1.0) for a in (WARD_HALF_LEN_M, 0.0065) for b in (1.0, -1.0)]
    cu = torch.tensor([e * a * cw - b * WARD_HALF_WIDTH_M * sw for e, a, b in corners], device=device)
    ct = torch.tensor([e * a * sw + b * WARD_HALF_WIDTH_M * cw for e, a, b in corners], device=device)
    add(cu.expand(n, len(corners)), ct.expand(n, len(corners)), (tip - c).expand(n, len(corners)))
    # The sides, at three depths behind the tip.
    side = torch.arange(12, device=device) * (2 * math.pi / 12)
    for back in (0.0025, 0.0055, 0.0090):
        h = torch.full((n, 1), KEY_LENGTH_M - back, device=device)
        add(SHAFT_R_M * torch.cos(side).expand(n, 12), SHAFT_R_M * torch.sin(side).expand(n, 12), h.expand(n, 12))
        add(cu.expand(n, len(corners)), ct.expand(n, len(corners)), h.expand(n, len(corners)))
    u, t, h = torch.cat(rows_u, 1), torch.cat(rows_t, 1), torch.cat(rows_h, 1)
    points = torch.stack((u, -t, KEY_BACK_LINK6_Z_M + h), dim=-1)
    return points, face


def keyway_sdf(x: torch.Tensor, y: torch.Tensor, hole_r: torch.Tensor, ward_l: torch.Tensor, ward_w: torch.Tensor):
    """The keyway's cross-section, the shaft's circle united with the wards' slot, in the lock's (u, t) plane: the signed
    distance (negative inside the keyway) and its in-plane gradient (out of the keyway), for points (N, P)."""
    rho = torch.sqrt(x * x + y * y).clamp(min=1e-9)
    d_circle = rho - hole_r
    g_circle = torch.stack((x / rho, y / rho), -1)
    cw, sw = math.cos(WARD_ANGLE_RAD), math.sin(WARD_ANGLE_RAD)
    a = x * cw + y * sw
    b = -x * sw + y * cw
    qa, qb = a.abs() - ward_l, b.abs() - ward_w
    oa, ob = qa.clamp(min=0.0), qb.clamp(min=0.0)
    outside = torch.sqrt(oa * oa + ob * ob)
    d_rect = outside + torch.minimum(torch.maximum(qa, qb), torch.zeros_like(qa))
    sa, sb = torch.sign(a) + (a == 0), torch.sign(b) + (b == 0)
    out_norm = outside.clamp(min=1e-9)
    ga_out, gb_out = oa * sa / out_norm, ob * sb / out_norm
    a_wins = qa > qb
    ga_in, gb_in = sa * a_wins, sb * (~a_wins)
    is_out = outside > 0
    ga, gb = torch.where(is_out, ga_out, ga_in), torch.where(is_out, gb_out, gb_in)
    g_rect = torch.stack((ga * cw - gb * sw, ga * sw + gb * cw), -1)
    rect = d_rect < d_circle
    d = torch.where(rect, d_rect, d_circle)
    g = torch.where(rect.unsqueeze(-1), g_rect, g_circle)
    return d, g


class KeyLock:
    """The cam lock in every environment and the key in every hand: per-environment frames and draws, and the contact."""

    def __init__(self, num_envs: int, device):
        self.n, self.device = num_envs, device
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        #: The lock's frame (env frame): the keyway's mouth on its face, its axis into the lock, its u and t.
        self.origin, self.axis, self.e_u, self.e_t = zeros(num_envs, 3), zeros(num_envs, 3), zeros(num_envs, 3), zeros(num_envs, 3)
        self.axis[:, 0] = 1.0
        self.e_u[:, 1] = -1.0
        self.e_t[:, 2] = 1.0
        self.play, self.lock_chamfer, self.mu = zeros(num_envs), zeros(num_envs), zeros(num_envs)
        self.key_chamfer = torch.full((num_envs,), 0.0005, device=device)
        self.grip_slip_n = torch.full((num_envs,), 30.0, device=device)
        self.slip_m = zeros(num_envs)
        self.points, self.face_points = key_points(self.key_chamfer)
        #: Each point's contact: 0 none, 1 the face, 2 the edge break, 3 the keyway's wall, 4 its floor.
        self.mode = torch.zeros(num_envs, self.points.shape[1], dtype=torch.long, device=device)
        self.k_point = CONTACT_K_N_PER_M / POINTS_SHARE
        self.c_point = CONTACT_C_N_S_PER_M / POINTS_SHARE
        # Outputs of the last step.
        self.force_w, self.torque_w = zeros(num_envs, 3), zeros(num_envs, 3)
        self.normal_n, self.face_n, self.wall_n, self.floor_n = zeros(num_envs), zeros(num_envs), zeros(num_envs), zeros(num_envs)
        self.axial_n = zeros(num_envs)
        self.tip_depth, self.tip_lateral = zeros(num_envs), zeros(num_envs, 2)
        self.tip_inside = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.peak_n = zeros(num_envs)

    @property
    def hole_r(self) -> torch.Tensor:
        return SHAFT_R_M + self.play

    def place(self, ids: torch.Tensor, origin, axis, e_u, play, lock_chamfer, key_chamfer, mu, grip_slip_n) -> None:
        """Mount locks for `ids`: the mouth's centre `origin`, `axis` into the lock, `e_u` the bar's way at insertion."""
        self.origin[ids], self.axis[ids] = origin, axis
        e_u = e_u - (e_u * axis).sum(-1, keepdim=True) * axis
        e_u = e_u / e_u.norm(dim=-1, keepdim=True)
        self.e_u[ids] = e_u
        # t = n x u with n = -axis (the key's frame, `geometry`), i.e. u x axis.
        self.e_t[ids] = torch.cross(e_u, axis, dim=-1)
        self.play[ids], self.lock_chamfer[ids], self.mu[ids] = play, lock_chamfer, mu
        self.key_chamfer[ids] = key_chamfer
        self.grip_slip_n[ids] = grip_slip_n
        self.slip_m[ids] = 0.0
        self.peak_n[ids] = 0.0
        self.mode[ids] = 0
        self.points[ids] = key_points(key_chamfer)[0]

    def to_lock(self, p: torch.Tensor) -> torch.Tensor:
        """Env-frame points (N, ..., 3) in the lock's (u, t, depth) coordinates."""
        shape = (self.n,) + (1,) * (p.dim() - 2) + (3,)
        rel = p - self.origin.view(shape)
        return torch.stack(((rel * self.e_u.view(shape)).sum(-1), (rel * self.e_t.view(shape)).sum(-1),
                            (rel * self.axis.view(shape)).sum(-1)), -1)

    def step(self, link6_pos: torch.Tensor, link6_rot: torch.Tensor, link6_vel: torch.Tensor, link6_omega: torch.Tensor,
             dt: float, active: torch.Tensor) -> None:
        """The contact for keys held by Link6 at `link6_pos` (env frame) and `link6_rot` (world, (N, 3, 3)), moving at
        `link6_vel`, `link6_omega` (world): sets `force_w`, `torque_w` (about the jaw centre) and the diagnostics."""
        pts_local = self.points.clone()
        pts_local[..., 2] = pts_local[..., 2] - self.slip_m.unsqueeze(-1)       # slid back in the jaws
        arm = torch.einsum("nij,npj->npi", link6_rot, pts_local)
        p = link6_pos.unsqueeze(1) + arm
        v = link6_vel.unsqueeze(1) + torch.cross(link6_omega.unsqueeze(1).expand_as(arm), arm, dim=-1)
        q = self.to_lock(p)
        x, y, z = q[..., 0], q[..., 1], q[..., 2]
        d, g = keyway_sdf(x, y, self.hole_r.unsqueeze(-1), (WARD_HALF_LEN_M + self.play).unsqueeze(-1),
                          (WARD_HALF_WIDTH_M + self.play).unsqueeze(-1))
        ch = self.lock_chamfer.unsqueeze(-1)
        grow = (ch - z).clamp(min=0.0)
        d_eff = d - grow
        behind = z > 0.0
        void = d_eff < 0.0
        floor = behind & void & (z > KEYWAY_DEPTH_M)
        metal = behind & ~void
        # The ways out of the metal: the face, the edge break (only within its depth), the keyway's wall (below it). A
        # point keeps the way it first went in by (`mode`) while it stays in the metal: the nearest way out of a point
        # pressed deep into the face beside the keyway is sideways into it, which would pull a key pressed on the face
        # into the keyway through a sharp edge.
        big = torch.full_like(z, 1e3)
        pen_face = torch.where(metal, z, big)
        in_break = metal & (z < ch)
        pen_break = torch.where(in_break, d_eff / math.sqrt(2.0), big)
        pen_wall = torch.where(metal & (z >= ch), d, big)
        nearest = torch.where((pen_face <= pen_break) & (pen_face <= pen_wall), 1,
                              torch.where(pen_break <= pen_wall, 2, 3))
        mode = self.mode
        mode = torch.where(floor, 4, mode)
        mode = torch.where(metal & ((mode == 0) | (mode == 4)), nearest, mode)
        mode = torch.where(metal & (mode == 2) & (z >= ch), 3, mode)
        mode = torch.where(metal & (mode == 3) & (z < ch), 2, mode)
        mode = torch.where(metal | floor, mode, 0)
        self.mode = mode
        use_face, use_break, use_wall = mode == 1, mode == 2, mode == 3
        touching = mode > 0
        pen = torch.where(use_face, z, torch.where(use_break, d_eff / math.sqrt(2.0),
                                                  torch.where(use_wall, d, z - KEYWAY_DEPTH_M)))
        pen = torch.where(touching, pen.clamp(min=0.0), torch.zeros_like(pen))
        # Normals, in the lock's (u, t, depth) coordinates: out of the metal, into free space or the keyway.
        zero = torch.zeros_like(z)
        n_face = torch.stack((zero, zero, -torch.ones_like(z)), -1)
        n_wall = torch.cat((-g, zero.unsqueeze(-1)), -1)
        n_break = torch.cat((-g, -torch.ones_like(z).unsqueeze(-1)), -1) / math.sqrt(2.0)
        normal_l = torch.where(use_break.unsqueeze(-1), n_break, torch.where(use_wall.unsqueeze(-1), n_wall, n_face))
        basis = torch.stack((self.e_u, self.e_t, self.axis), dim=1)            # (N, 3, 3): rows u, t, axis
        normal = torch.einsum("npk,nkj->npj", normal_l, basis)
        v_n = (v * normal).sum(-1)
        f_n = (self.k_point * pen - self.c_point * v_n).clamp(min=0.0, max=POINT_MAX_N) * touching
        v_t = v - v_n.unsqueeze(-1) * normal
        speed = v_t.norm(dim=-1, keepdim=True)
        f_t = -self.mu.view(-1, 1, 1) * f_n.unsqueeze(-1) * v_t / speed.clamp(min=FRICTION_V_REG_M_S)
        f = f_n.unsqueeze(-1) * normal + f_t
        f = f * active.view(-1, 1, 1)
        jaw = link6_pos + link6_rot[:, :, 2] * JAW_LINK6_Z_M
        self.force_w = f.sum(1)
        self.torque_w = torch.cross(p - jaw.unsqueeze(1), f, dim=-1).sum(1)
        on = active.float()
        self.normal_n = f_n.sum(1) * on
        self.face_n = (f_n * use_face).sum(1) * on
        self.wall_n = (f_n * (use_wall | use_break)).sum(1) * on
        self.floor_n = (f_n * floor).sum(1) * on
        self.axial_n = -(self.force_w * self.axis).sum(-1)           # pushing the key back out, along the axis
        self.peak_n = torch.maximum(self.peak_n, self.force_w.norm(dim=-1) * on)
        # The jaws slip under an axial push past their grip.
        excess = (self.axial_n - self.grip_slip_n).clamp(min=0.0) * on
        self.slip_m = (self.slip_m + SLIP_RATE_M_S_N * excess * dt).clamp(max=SLIP_LOST_M + 0.003)
        # Where the tip is: the shaft's axis at the tip, in the lock's coordinates.
        tip_local = torch.zeros(self.n, 1, 3, device=self.device)
        tip_local[..., 2] = KEY_BACK_LINK6_Z_M + KEY_LENGTH_M - self.slip_m.unsqueeze(-1)
        tip = link6_pos.unsqueeze(1) + torch.einsum("nij,npj->npi", link6_rot, tip_local)
        tq = self.to_lock(tip)[:, 0]
        self.tip_depth = tq[:, 2]
        self.tip_lateral = tq[:, :2]
        # In the keyway: the shaft's axis within the play of the keyway's, plus the penalty contact's give (a key truly in
        # sat 0.2-0.7 mm off, 2026-11-13), and deeper than a key resting on the face sinks into it.
        self.tip_inside = (tq[:, :2].norm(dim=-1) < self.play + INSIDE_GIVE_M) & (self.tip_depth > INSIDE_DEPTH_M)

    @property
    def seated(self) -> torch.Tensor:
        """The key in, its tip within `SEATED_WITHIN_M` of the keyway's floor."""
        return self.tip_inside & (self.tip_depth > KEYWAY_DEPTH_M - SEATED_WITHIN_M)

    @property
    def lost(self) -> torch.Tensor:
        return self.slip_m > SLIP_LOST_M


# --- the task layer ---------------------------------------------------------------------------------------------------

APPROACH, SEARCH, SEAT, HOLD, DONE = 0, 1, 2, 3, 4
#: In along the estimated axis this fast while the reference is more than `FAST_UNTIL_M` short of the estimated face,
#: then slower (the dog program: 6 and 2 mm/s, `insert_plan.FAST_IN_SPEED_M_S`).
FAST_SPEED_M_S = 0.008
FAST_UNTIL_M = 0.004
SLOW_SPEED_M_S = 0.004
#: The reference goes no further than this past the estimated seat (the dog's 40 mm, `IN_SLACK_M`, for its lidar's
#: depth after lying down; here the estimate's drawn error is within 10 mm).
SLACK_M = 0.014
#: The face felt: the hand this far behind its reference along the axis, for this long (`insert_plan.CONTACT_LAG_M`).
CONTACT_LAG_M = 0.003
CONTACT_HOLD_S = 0.3
#: ... further behind than it was over the approach's first `BASE_S` (the arm's own lag along the axis: the dog program's
#: baseline, `insert_plan._stopped`), or this far behind whatever the baseline (`insert_plan.FAST_LAG_MAX_M`).
BASE_S = 0.6
LAG_MAX_M = 0.008
#: Position-only: the press, as how far into the face the reference is held (`insert_plan.PRESS_M`).
PRESS_M = 0.002
#: The search: an Archimedean spiral across the face at the touched depth, this far between turns, this fast along it,
#: out to this radius and back in (the dog's 0.4 mm, 1.5 mm/s, 4 mm: its keyway had 0.4 mm of play and no break).
SPIRAL_PITCH_M = 0.0012
SPIRAL_SPEED_M_S = 0.005
SPIRAL_R_M = 0.0045
#: The wrist's roll about the shaft while searching (`insert_plan.WRIST_SEARCH_DEG`, a sine of period 2 s).
WIGGLE_DEG = 8.0
WIGGLE_PERIOD_S = 2.0
#: In: the hand this far past where it touched (`insert_plan.ENTER_M`), or held sideways this far off the spiral (the
#: keyway's walls hold a key that is in; the face lets it slide), for `CONTACT_HOLD_S`.
ENTER_M = 0.0015
HELD_SIDEWAYS_M = 0.003
#: The feel after the touch (`PROBE_S`): the reference moved `PROBE_M` across the axis (along u). On the face the hand
#: follows it; in the keyway (a key that went straight in and touched the floor, or one partly in) its walls hold it, the
#: hand moving less than `PROBE_HELD_M`: then it is in. Round 3: a key that went straight in waited 5 s for the spiral
#: to reach the 3 mm "held" cue.
PROBE_S = 0.6
PROBE_M = 0.003
PROBE_HELD_M = 0.0008
#: Opt-in (`InsertLayer.light_probe`, `--insert_light_probe`): the press eased through the probe, position-only's reference
#: back to the touch and force-aware's press halved. Off for every number before 2026-11-13 06:00: round 5's position-only
#: `7800` pressed so hard (median peak 25 N) that friction held the hand against the probe, read as held, 94 false seats.
#: Seated: past where it touched by the keyway's depth less this, and still (`SETTLE_SPEED_M_S`) for `CONTACT_HOLD_S`.
SEATED_SHORT_M = 0.002
#: ... or still this long at any depth once in (a key that went straight in touched the floor, not the face).
STILL_ANY_S = 1.5
#: A key stopped short of the floor that long is jiggled: the wrist rolled +-`JIGGLE_DEG` about where it went in (period
#: `JIGGLE_PERIOD_S`), the press up by half (force-aware) or the reference 1 mm deeper (position-only), for at most
#: `JIGGLE_S`; then the layer holds it where it is. Round 3 at its start: keys stopped 3-7 mm in were held there.
JIGGLE_DEG = 3.0
JIGGLE_PERIOD_S = 1.0
JIGGLE_S = 4.0
JIGGLE_HELD_S = 1.5
SETTLE_SPEED_M_S = 0.002
#: The depth's low-pass for that speed (s): the reports are quantised to 0.1 deg and come at 9 Hz.
SETTLE_FILTER_S = 0.15
HOLD_S = 1.0
#: Force-aware: the press while searching and seating (N, drawn in training), held lighter once seated; the reward's
#: compliance (N/m) along the axis and across it, which sets the goal back from the reference by the force.
PRESS_N = (5.0, 10.0)
EVAL_PRESS_N = 8.0
HOLD_PRESS_N = 3.0
K_AXIAL_N_PER_M = 800.0
K_LATERAL_N_PER_M = 3000.0


class InsertLayer:
    """The dog's insertion on the arm's reported joints, per environment, in the *estimated* lock's coordinates: the
    tip's reference (depth along the axis from the estimated face, and across it), the wrist's roll, and the press."""

    def __init__(self, num_envs: int, device):
        self.n, self.device = num_envs, device
        self.light_probe = False
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        self.phase = torch.full((num_envs,), DONE, dtype=torch.long, device=device)
        self.force_law = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.ref_depth, self.ref_lateral = zeros(num_envs), zeros(num_envs, 2)
        self.touch_depth = zeros(num_envs)
        self.press_n, self.force_cmd = zeros(num_envs), zeros(num_envs)
        self.roll = zeros(num_envs)
        self.spiral_s, self.spiral_dir = zeros(num_envs), torch.ones(num_envs, device=device)
        self.wiggle_t = zeros(num_envs)
        self.lag_for, self.held_for, self.still_for, self.phase_for = zeros(num_envs), zeros(num_envs), zeros(num_envs), zeros(num_envs)
        self.last_depth = zeros(num_envs)
        self.seat_end = zeros(num_envs)
        self.still_any = zeros(num_envs)
        self.jiggle_for, self.entry_roll = zeros(num_envs), zeros(num_envs)
        self.probe_from = zeros(num_envs, 2)
        self.lag_base = zeros(num_envs)
        self.touched = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.entered_by = zeros(num_envs)                     # 1: advanced, 2: held sideways
        self.max_depth = torch.full((num_envs,), KEYWAY_DEPTH_M + SLACK_M, device=device)

    def start(self, ids: torch.Tensor, start_depth: torch.Tensor, force_law: torch.Tensor, press_n: torch.Tensor) -> None:
        self.phase[ids] = APPROACH
        self.force_law[ids] = force_law
        self.ref_depth[ids], self.ref_lateral[ids] = start_depth, 0.0
        self.touch_depth[ids] = 0.0
        self.press_n[ids] = press_n
        # The approach is by position for both: the force-aware press starts once the face is felt (round 1 commanded it
        # from the start, and a hand that did not yet lead its goal by the press read as touching in free space).
        self.force_cmd[ids] = 0.0
        self.roll[ids] = 0.0
        self.spiral_s[ids], self.spiral_dir[ids], self.wiggle_t[ids] = 0.0, 1.0, 0.0
        for buf in (self.lag_for, self.held_for, self.still_for, self.phase_for):
            buf[ids] = 0.0
        self.last_depth[ids] = start_depth
        self.seat_end[ids] = 0.0
        self.still_any[ids] = 0.0
        self.jiggle_for[ids], self.entry_roll[ids] = 0.0, 0.0
        self.probe_from[ids] = 0.0
        self.lag_base[ids] = 0.0
        self.touched[ids] = False
        self.entered_by[ids] = 0.0

    def stop(self, ids: torch.Tensor) -> None:
        self.phase[ids] = DONE
        self.force_cmd[ids] = 0.0

    def free_lead(self) -> torch.Tensor:
        """How far past the goal the force-aware hand sits in free space (the press over the axial compliance)."""
        return self.force_cmd / K_AXIAL_N_PER_M

    def goal(self) -> tuple[torch.Tensor, torch.Tensor]:
        """The goal the policy is given for the tip: the reference, set back by the force-aware press's lead."""
        return self.ref_depth - self.free_lead(), self.ref_lateral

    def _spiral(self, s: torch.Tensor) -> torch.Tensor:
        """The point `s` along an Archimedean spiral r = b theta (b = pitch / 2 pi), by its arc length ~ b theta^2 / 2."""
        b = SPIRAL_PITCH_M / (2 * math.pi)
        theta = torch.sqrt(2.0 * s.clamp(min=0.0) / b)
        r = b * theta
        return torch.stack((r * torch.cos(theta), r * torch.sin(theta)), -1)

    def step(self, depth: torch.Tensor, lateral: torch.Tensor, dt: float) -> None:
        """Advance with the tip measured at `depth` and `lateral` (from the arm's reported joints)."""
        active = self.phase < DONE
        if not bool(active.any()):
            return
        filtered = self.last_depth + (depth - self.last_depth) * min(1.0, dt / SETTLE_FILTER_S)
        speed = (filtered - self.last_depth) / dt
        self.last_depth = filtered
        self.phase_for = torch.where(active, self.phase_for + dt, self.phase_for)
        lag = self.ref_depth - depth
        # Approach: in along the axis until the face is felt.
        app = active & (self.phase == APPROACH)
        v = torch.where(self.ref_depth < -FAST_UNTIL_M, torch.full_like(depth, FAST_SPEED_M_S),
                        torch.full_like(depth, SLOW_SPEED_M_S))
        self.ref_depth = torch.where(app, (self.ref_depth + v * dt).clamp(max=self.max_depth), self.ref_depth)
        early = app & (self.phase_for < BASE_S)
        self.lag_base = torch.where(early, (self.lag_base + (lag - self.lag_base) * min(1.0, dt / 0.2)).clamp(0.0, 0.005),
                                    self.lag_base)
        behind = (~early & (lag - self.lag_base > CONTACT_LAG_M)) | (lag > LAG_MAX_M)
        self.lag_for = torch.where(app & behind, self.lag_for + dt, torch.zeros_like(self.lag_for))
        felt = app & (self.lag_for >= CONTACT_HOLD_S)
        if bool(felt.any()):
            self.touch_depth = torch.where(felt, depth, self.touch_depth)
            self.touched |= felt
            press = torch.where(self.force_law, torch.zeros_like(depth), torch.full_like(depth, PRESS_M))
            self.force_cmd = torch.where(felt & self.force_law, self.press_n, self.force_cmd)
            self.ref_depth = torch.where(felt, depth + press + self.free_lead(), self.ref_depth)
            self.phase = torch.where(felt, torch.full_like(self.phase, SEARCH), self.phase)
            self.phase_for = torch.where(felt, torch.zeros_like(self.phase_for), self.phase_for)
            self.held_for = torch.where(felt, torch.zeros_like(self.held_for), self.held_for)
            self.probe_from = torch.where(felt.unsqueeze(-1), lateral, self.probe_from)
        # Search: the spiral across the face, the wrist rolling, the key pressed on.
        srch = active & (self.phase == SEARCH)
        if bool(srch.any()):
            # First the feel: the reference across by `PROBE_M`, then the spiral from where it started.
            probing = srch & (self.phase_for < PROBE_S)
            probe_ref = self.ref_lateral.clone()
            probe_ref[:, 0] = PROBE_M * (self.phase_for / (0.5 * PROBE_S)).clamp(max=1.0)
            spiraling = srch & ~probing
            self.spiral_s = torch.where(spiraling, self.spiral_s + self.spiral_dir * SPIRAL_SPEED_M_S * dt, self.spiral_s)
            r_max_s = 0.5 * SPIRAL_R_M ** 2 * 2 * math.pi / SPIRAL_PITCH_M
            self.spiral_dir = torch.where(spiraling & (self.spiral_s >= r_max_s), -torch.ones_like(self.spiral_dir),
                                          torch.where(spiraling & (self.spiral_s <= 0.0), torch.ones_like(self.spiral_dir),
                                                      self.spiral_dir))
            self.ref_lateral = torch.where(probing.unsqueeze(-1), probe_ref,
                                           torch.where(spiraling.unsqueeze(-1), self._spiral(self.spiral_s), self.ref_lateral))
            self.wiggle_t = torch.where(spiraling, self.wiggle_t + dt, self.wiggle_t)
            self.roll = torch.where(spiraling, math.radians(WIGGLE_DEG) * torch.sin(2 * math.pi * self.wiggle_t / WIGGLE_PERIOD_S),
                                    self.roll)
            if self.light_probe:
                eased = probing & (self.phase_for < 1.5 * dt)
                resumed = srch & (self.phase_for >= PROBE_S) & (self.phase_for < PROBE_S + 1.5 * dt)
                # Position-only: the reference back to the touch, then pressed again; force-aware: the press halved.
                self.ref_depth = torch.where(eased & ~self.force_law, self.touch_depth, self.ref_depth)
                self.ref_depth = torch.where(resumed & ~self.force_law, self.touch_depth + PRESS_M, self.ref_depth)
                half = eased & self.force_law
                self.ref_depth = torch.where(half, self.ref_depth - 0.5 * self.press_n / K_AXIAL_N_PER_M, self.ref_depth)
                self.force_cmd = torch.where(half, 0.5 * self.press_n, self.force_cmd)
                full = resumed & self.force_law
                self.force_cmd = torch.where(full, self.press_n, self.force_cmd)
                self.ref_depth = torch.where(full, self.ref_depth + 0.5 * self.press_n / K_AXIAL_N_PER_M, self.ref_depth)
            probe_end = srch & (self.phase_for >= PROBE_S) & (self.phase_for < PROBE_S + 1.5 * dt)
            probe_held = probe_end & ((lateral - self.probe_from)[:, 0].abs() < PROBE_HELD_M)
            advanced = srch & (depth > self.touch_depth + ENTER_M)
            held = spiraling & ((lateral - self.ref_lateral).norm(dim=-1) > HELD_SIDEWAYS_M)
            self.held_for = torch.where(held, self.held_for + dt, torch.zeros_like(self.held_for))
            held_long = spiraling & (self.held_for >= CONTACT_HOLD_S)
            entered = advanced | held_long | probe_held
            if bool(entered.any()):
                self.entered_by = torch.where(entered, torch.where(advanced, 1.0, 2.0), self.entered_by)
                # The key is where the keyway is: stop pulling it across, hold the wrist where it went in.
                self.ref_lateral = torch.where(entered.unsqueeze(-1), lateral, self.ref_lateral)
                self.phase = torch.where(entered, torch.full_like(self.phase, SEAT), self.phase)
                self.phase_for = torch.where(entered, torch.zeros_like(self.phase_for), self.phase_for)
                self.still_for = torch.where(entered, torch.zeros_like(self.still_for), self.still_for)
                # On to the floor the touch implies, either way in: a key held sideways is in the keyway, but may be only
                # partly in (round 2 at +200: 64 of 68 false seats were keys 3-8 mm in, seated where they were held).
                seat_target = self.touch_depth + KEYWAY_DEPTH_M
                seat_end = seat_target + torch.where(self.force_law, 0.0, PRESS_M)
                self.seat_end = torch.where(entered, torch.maximum(seat_end, self.ref_depth), self.seat_end)
                self.ref_depth = torch.where(entered & self.force_law, seat_target + self.free_lead(), self.ref_depth)
                self.ref_depth = torch.where(entered & ~self.force_law, depth.maximum(self.ref_depth), self.ref_depth)
                self.still_any = torch.where(entered, torch.zeros_like(self.still_any), self.still_any)
                self.jiggle_for = torch.where(entered, torch.zeros_like(self.jiggle_for), self.jiggle_for)
                self.entry_roll = torch.where(entered, self.roll, self.entry_roll)
        # Seat: on in until it stops past where it should.
        seat = active & (self.phase == SEAT)
        if bool(seat.any()):
            nxt = torch.minimum(self.ref_depth + SLOW_SPEED_M_S * dt, self.seat_end)
            self.ref_depth = torch.where(seat & ~self.force_law, nxt, self.ref_depth)
            deep = depth > self.touch_depth + KEYWAY_DEPTH_M - SEATED_SHORT_M
            # A key found in by its hold (`entered_by` 2) may have touched the floor or be partly in: it is jiggled for
            # `JIGGLE_HELD_S` first, then taken as seated once still.
            held_in = seat & (self.entered_by == 2.0)
            calm = seat & (speed.abs() < SETTLE_SPEED_M_S)
            self.still_for = torch.where(calm & deep, self.still_for + dt, torch.zeros_like(self.still_for))
            self.still_any = torch.where(calm, self.still_any + dt, torch.zeros_like(self.still_any))
            # Stuck short of the floor: jiggle it, pressing harder, until it goes on in or `JIGGLE_S` runs out.
            jiggling = seat & ~deep & (((self.still_any >= STILL_ANY_S) | (self.jiggle_for > 0.0))
                                       | (held_in & (self.jiggle_for < JIGGLE_HELD_S)))
            self.jiggle_for = torch.where(jiggling, self.jiggle_for + dt, self.jiggle_for)
            wob = math.radians(JIGGLE_DEG) * torch.sin(2 * math.pi * self.jiggle_for / JIGGLE_PERIOD_S)
            self.roll = torch.where(jiggling, self.entry_roll + wob, torch.where(seat & deep, self.entry_roll, self.roll))
            started = jiggling & (self.force_cmd <= self.press_n + 1e-6) & self.force_law
            self.force_cmd = torch.where(started, 1.5 * self.press_n, self.force_cmd)
            self.ref_depth = torch.where(started, self.ref_depth + 0.5 * self.press_n / K_AXIAL_N_PER_M, self.ref_depth)
            self.seat_end = torch.where(jiggling & ~self.force_law & (self.jiggle_for <= dt + 1e-6), self.seat_end + 0.001,
                                        self.seat_end)
            seated = seat & ((self.still_for >= CONTACT_HOLD_S) | (self.jiggle_for >= JIGGLE_S)
                             | ((self.still_any >= STILL_ANY_S) & deep)
                             | (held_in & (self.jiggle_for >= JIGGLE_HELD_S) & (self.still_any >= CONTACT_HOLD_S)))
            if bool(seated.any()):
                self.phase = torch.where(seated, torch.full_like(self.phase, HOLD), self.phase)
                self.phase_for = torch.where(seated, torch.zeros_like(self.phase_for), self.phase_for)
                self.force_cmd = torch.where(seated & self.force_law, torch.full_like(self.force_cmd, HOLD_PRESS_N),
                                             self.force_cmd)
                self.ref_depth = torch.where(seated, depth + torch.where(self.force_law, self.free_lead(),
                                                                         torch.zeros_like(depth)), self.ref_depth)
        hold = active & (self.phase == HOLD) & (self.phase_for >= HOLD_S)
        self.phase = torch.where(hold, torch.full_like(self.phase, DONE), self.phase)

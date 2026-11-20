"""The held mechanism's viewer overlay (`mech_env.py`): the mechanism exists only as `mechanism.Mechanism`'s plant.

The mechanism is built round the tool when the grasp closes, and is drawn from then on, per environment:

  * a **slide** (drawer, latch, bolt) pulled toward the robot: the **cabinet** (dark) behind the closed handle, the
    **drawer front** at the handle and the **drawer's sides** between them, so the travel shows as the drawer coming out;
  * a **slide** pushed away from the robot (a button): the **housing** (dark) beyond the travel, and the **cap** at the
    handle, moving into it;
  * a **hinge** (door, lid): the **leaf** from the hinge's axis to just past the handle, the **hinge pin** (dark) on the
    axis, and the **doorway** (dark), where the leaf was closed, which the leaf uncovers as it opens;
  * the **handle**, a dark bar where the tool holds it (a slide's across its front, a hinge's along the axis);
  * the **path** the handle may travel, closed to fully open, as thin grey segments;
  * **yellow**: where the travel goal's reference puts the handle now (`TravelGoal.ref`);
  * **a red bar**: the mechanism's force on the tool, from the handle, `FORCE_M_PER_N` long a newton.

The moving parts (front, sides, leaf, cap) are coloured by state: **orange** held and opening, **green** at least
`mech_cfg.EVAL_OPENED_FRACTION` of the travel open (how `mech_eval` scores "opened"), **red** while the drive is above
`mech_cfg.BUDGET_N` or on the step the handle tore out. Nothing is drawn before the grasp or for an environment without a
mechanism. The sizes are drawn, not the plant's: the plant is a handle on a path (`mechanism.py`); nothing else of it
exists. Viewer only: nothing is allocated until a viewer asks for it.
"""
from __future__ import annotations

import torch

from . import mech_cfg
from .debug_vis import arrow_pose
from .leg_vis import _frame_quat

#: The handle stands this far proud of the front or the leaf, m.
PROUD_M = 0.03
PLATE_M = 0.02
#: A drawer's front (across, up) and its cabinet's depth behind the closed front, m.
FRONT_M = (0.30, 0.18)
CABINET_DEPTH_M = 0.20
CABINET_MARGIN_M = 0.04
#: A button's cap (along, across) and its housing (along, across, up), m.
CAP_M = (0.02, 0.05)
HOUSING_M = (0.10, 0.16, 0.16)
#: A hinge's leaf: past the handle, and along the axis, m.
LEAF_PAST_M = 0.05
LEAF_HEIGHT_M = 0.40
PIN_R_M = 0.012
HANDLE_R_M = 0.010
HANDLE_LEN_M = 0.10
PATH_R_M = 0.003
PATH_SEGMENTS = 12
GOAL_R_M = 0.012
FORCE_M_PER_N = 0.003
FORCE_R_M = 0.005
#: Below this force on the tool, N, the arrow is hidden.
FORCE_SHOWN_N = 1.0
HIDDEN = 1.0e-6

GREY, ORANGE, GREEN, RED = range(4)
COLOURS = ((0.62, 0.62, 0.66), (1.0, 0.5, 0.05), (0.1, 0.75, 0.25), (0.9, 0.1, 0.1))
FIXED, PIN, HANDLE, GOAL, FORCE, PATH = range(6)
BODY = 6                                   # + colour
FIXED_SLOT, FRONT_SLOT, SIDES_SLOT, HANDLE_SLOT, PIN_SLOT, GOAL_SLOT, FORCE_SLOT = range(7)
PATH_SLOT = 7
MARKERS_PER_ENV = PATH_SLOT + PATH_SEGMENTS


def marker_cfg(prim_path: str):
    import isaaclab.sim as sim_utils
    from isaaclab.markers import VisualizationMarkersCfg

    def material(colour):
        return sim_utils.PreviewSurfaceCfg(diffuse_color=colour)

    def block(colour):
        return sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=material(colour))

    def rod(colour):
        return sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="X", visual_material=material(colour))

    markers = {
        "fixed": block((0.22, 0.22, 0.24)),
        "pin": rod((0.15, 0.15, 0.17)),
        "handle": rod((0.12, 0.12, 0.13)),
        "goal": sim_utils.SphereCfg(radius=1.0, visual_material=material((1.0, 0.9, 0.0))),
        "force": rod((0.9, 0.1, 0.1)),
        "path": rod((0.55, 0.57, 0.62)),
    }
    for k, colour in enumerate(COLOURS):
        markers[f"body_{k}"] = block(colour)
    return VisualizationMarkersCfg(prim_path=prim_path, markers=markers)


def _unit(v: torch.Tensor) -> torch.Tensor:
    return v / v.norm(dim=-1, keepdim=True).clamp(min=1e-9)


def _across(x: torch.Tensor) -> torch.Tensor:
    """A unit vector perpendicular to `x`, horizontal where `x` is not near vertical."""
    up = torch.zeros_like(x)
    up[:, 2] = 1.0
    ahead = torch.zeros_like(x)
    ahead[:, 0] = 1.0
    ref = torch.where((x * up).sum(-1, keepdim=True).abs() < 0.95, up, ahead)
    return _unit(torch.cross(ref, x, dim=-1))


def state_colour(m, opened_fraction: float = mech_cfg.EVAL_OPENED_FRACTION,
                 budget_n: float | None = None) -> torch.Tensor:
    """The moving parts' colour code per environment: orange held, green opened, red over the budget or torn."""
    budget = mech_cfg.BUDGET_N if budget_n is None else budget_n
    colour = torch.full_like(m.s, ORANGE, dtype=torch.long)
    colour = torch.where(m.s >= opened_fraction * m.travel, torch.full_like(colour, GREEN), colour)
    return torch.where((m.drive_n > budget) | m.torn, torch.full_like(colour, RED), colour)


def marker_poses(m, base: torch.Tensor, ref_s: torch.Tensor, shown: torch.Tensor,
                 colour: torch.Tensor | None = None) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Every marker's (translation, orientation, scale, prototype index), (N, MARKERS_PER_ENV, ...), in the env frame.

    `m` is the `Mechanism`, `base` the robot's base in the env frame (which side is the robot's), `ref_s` the travel
    goal's reference along the path, `shown` which environments draw anything. Pure torch, so it is tested without a
    simulator (`tests/test_mech_vis.py`)."""
    n, dev = m.s.shape[0], m.s.device
    t = torch.zeros(n, MARKERS_PER_ENV, 3, device=dev)
    q = torch.zeros(n, MARKERS_PER_ENV, 4, device=dev)
    q[..., 0] = 1.0
    s = torch.full((n, MARKERS_PER_ENV, 3), HIDDEN, device=dev)
    idx = torch.zeros(n, MARKERS_PER_ENV, dtype=torch.long, device=dev)
    code = lambda value: torch.full((n,), value, dtype=torch.long, device=dev)
    col = lambda *v: torch.tensor(v, device=dev, dtype=t.dtype).expand(n, len(v))
    colour = state_colour(m) if colour is None else colour
    hinge = m.hinge
    h = hinge.unsqueeze(-1)
    p = m.position()

    # --- a slide's frame: x along the opening, y across, z up --------------------------------------------------
    d = _unit(m.direction)
    toward = ((d * (base - m.start)).sum(-1) >= 0.0).unsqueeze(-1)      # pulled toward the robot (or sideways)
    y_s = _across(d)
    z_s = torch.cross(d, y_s, dim=-1)
    quat_s = _frame_quat(d, y_s, z_s)
    # --- a hinge's frame: x along the leaf now, y along the axis, z its normal, n toward the robot -------------
    r = m.radius.clamp(min=1e-6).unsqueeze(-1)
    x_h = _unit(p - m.centre)
    y_h = _unit(m.axis)
    z_h = torch.cross(x_h, y_h, dim=-1)
    n_h = torch.where(((z_h * (base - p)).sum(-1) >= 0.0).unsqueeze(-1), z_h, -z_h)
    quat_h = _frame_quat(x_h, y_h, z_h)
    x0 = _unit(m.radial)
    z0 = torch.cross(x0, y_h, dim=-1)
    n0 = torch.where(((z0 * (base - m.start)).sum(-1) >= 0.0).unsqueeze(-1), z0, -z0)
    quat_h0 = _frame_quat(x0, y_h, z0)

    # The fixed part: a pulled slide's cabinet behind the closed front, a pushed slide's housing beyond the travel,
    # a hinge's doorway where the leaf was closed.
    behind = PROUD_M + PLATE_M
    travel = m.travel.unsqueeze(-1)
    cabinet = m.start - d * (behind + 0.5 * CABINET_DEPTH_M)
    housing = m.start + d * (CAP_M[0] + travel + 0.5 * HOUSING_M[0])
    leaf_len = m.radius + LEAF_PAST_M
    doorway = m.centre + x0 * (0.5 * leaf_len).unsqueeze(-1) - n0 * (behind + 0.006)
    t[:, FIXED_SLOT] = torch.where(h, doorway, torch.where(toward, cabinet, housing))
    q[:, FIXED_SLOT] = torch.where(h, quat_h0, quat_s)
    s[:, FIXED_SLOT] = torch.where(
        h, torch.stack((leaf_len + 0.02, torch.full_like(leaf_len, LEAF_HEIGHT_M + 0.02), torch.full_like(leaf_len, 0.004)), -1),
        torch.where(toward, col(CABINET_DEPTH_M, FRONT_M[0] + CABINET_MARGIN_M, FRONT_M[1] + CABINET_MARGIN_M),
                    col(*HOUSING_M)))
    idx[:, FIXED_SLOT] = code(FIXED)

    # The moving part: a drawer's front, a button's cap, a hinge's leaf; coloured by state.
    front = p - d * (PROUD_M + 0.5 * PLATE_M)
    cap = p + d * (0.5 * CAP_M[0])
    leaf = m.centre + x_h * (0.5 * leaf_len).unsqueeze(-1) - n_h * (PROUD_M + 0.5 * PLATE_M)
    t[:, FRONT_SLOT] = torch.where(h, leaf, torch.where(toward, front, cap))
    q[:, FRONT_SLOT] = torch.where(h, quat_h, quat_s)
    s[:, FRONT_SLOT] = torch.where(
        h, torch.stack((leaf_len, torch.full_like(leaf_len, LEAF_HEIGHT_M), torch.full_like(leaf_len, PLATE_M)), -1),
        torch.where(toward, col(PLATE_M, *FRONT_M), col(CAP_M[0], CAP_M[1], CAP_M[1])))
    idx[:, FRONT_SLOT] = BODY + colour

    # A drawer's sides: from the cabinet's face to the front, as long as it is open.
    face = m.start - d * behind
    open_s = m.s.clamp(min=0.0).unsqueeze(-1)
    t[:, SIDES_SLOT] = face + 0.5 * open_s * d
    q[:, SIDES_SLOT] = quat_s
    sides = torch.stack((m.s.clamp(min=HIDDEN), torch.full_like(m.s, FRONT_M[0] - 0.04),
                         torch.full_like(m.s, FRONT_M[1] - 0.04)), -1)
    s[:, SIDES_SLOT] = torch.where(h | ~toward | (m.s < 0.002).unsqueeze(-1), torch.full_like(sides, HIDDEN), sides)
    idx[:, SIDES_SLOT] = BODY + colour

    # The handle: across a drawer's front, along a hinge's axis; a button has its cap.
    quat_bar_s = _frame_quat(y_s, z_s, d)
    quat_bar_h = _frame_quat(y_h, z_h, x_h)
    t[:, HANDLE_SLOT] = p
    q[:, HANDLE_SLOT] = torch.where(h, quat_bar_h, quat_bar_s)
    bar = col(HANDLE_LEN_M, HANDLE_R_M, HANDLE_R_M)
    s[:, HANDLE_SLOT] = torch.where(~h & ~toward, torch.full_like(bar, HIDDEN), bar)
    idx[:, HANDLE_SLOT] = code(HANDLE)

    # A hinge's pin, on the axis level with the leaf.
    t[:, PIN_SLOT] = m.centre - n_h * (PROUD_M + 0.5 * PLATE_M)
    q[:, PIN_SLOT] = quat_bar_h
    pin = col(LEAF_HEIGHT_M + 0.04, PIN_R_M, PIN_R_M)
    s[:, PIN_SLOT] = torch.where(h, pin, torch.full_like(pin, HIDDEN))
    idx[:, PIN_SLOT] = code(PIN)

    # The travel goal's reference, on the path.
    t[:, GOAL_SLOT] = m.position(torch.minimum(ref_s.clamp(min=0.0), m.travel))
    s[:, GOAL_SLOT] = col(GOAL_R_M, GOAL_R_M, GOAL_R_M)
    idx[:, GOAL_SLOT] = code(GOAL)

    # The mechanism's force on the tool, from the handle.
    f = m.force_on_tool * FORCE_M_PER_N
    quat_f, flen = arrow_pose(f)
    t[:, FORCE_SLOT] = p + 0.5 * f
    q[:, FORCE_SLOT] = quat_f
    arrow = torch.stack((flen, torch.full_like(flen, FORCE_R_M), torch.full_like(flen, FORCE_R_M)), -1)
    s[:, FORCE_SLOT] = torch.where((m.force_on_tool.norm(dim=-1) > FORCE_SHOWN_N).unsqueeze(-1), arrow,
                                   torch.full_like(arrow, HIDDEN))
    idx[:, FORCE_SLOT] = code(FORCE)

    # The path, closed to fully open.
    for k in range(PATH_SEGMENTS):
        a = m.position(m.travel * (k / PATH_SEGMENTS))
        b = m.position(m.travel * ((k + 1) / PATH_SEGMENTS))
        quat_k, length = arrow_pose(b - a)
        t[:, PATH_SLOT + k] = 0.5 * (a + b)
        q[:, PATH_SLOT + k] = quat_k
        s[:, PATH_SLOT + k] = torch.stack((length, torch.full_like(length, PATH_R_M), torch.full_like(length, PATH_R_M)), -1)
        idx[:, PATH_SLOT + k] = code(PATH)

    s[~shown] = HIDDEN
    return t, q, s, idx


class MechMarkers:
    def __init__(self, prim_path: str, num_envs: int, device: str) -> None:
        from isaaclab.markers import VisualizationMarkers

        self.markers = VisualizationMarkers(marker_cfg(prim_path))
        self.num_envs, self.device = num_envs, device

    def update(self, env) -> None:
        m = env._mech
        origins = env.scene.env_origins
        base = env._robot.data.root_pos_w - origins
        shown = env._mech_env & (m.grasped | m.torn)
        t, q, s, idx = marker_poses(m, base, env._travel_goal.ref, shown)
        t = t + origins.unsqueeze(1)
        self.markers.visualize(translations=t.view(-1, 3), orientations=q.view(-1, 4), scales=s.view(-1, 3),
                               marker_indices=idx.view(-1))

    def set_visibility(self, visible: bool) -> None:
        self.markers.set_visibility(visible)

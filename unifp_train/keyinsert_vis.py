"""The key insertion's viewer overlay (`keyinsert_env.py`): the lock and the key exist only in `keylock`'s torch plant.

The key hold's overlay (`leg_vis.HandleMarkers`) draws its key seated in the lock; here the key is in the jaws, so the
insertion draws its own, per environment with a lock:

  * **the door**, `keylock.LOCK_FACE_OFF_DOOR_M` behind the true lock's face, and **the lock's chrome cylinder** (the
    27.5 mm bezel) standing proud of it, both on the true lock's axis;
  * **the keyway** on its face in black, as the plant has it: the shaft's circle and the wards' slot, play included;
  * **the key in the jaws** (Link6, slid back by the grip's slip): its shaft and wards, its bar and the foot's
    platform, **grey** before the face is felt, **orange** searching, **blue** seating and holding, **green** seated
    (the plant's test), **red** once lost in the jaws or damaged;
  * **yellow**: where the robot believes the keyway is (the estimate's centre), and **magenta**: the layer's reference
    for the tip (the search's spiral). Both are drawn on the true face, 0.6 mm in front of it, wherever their depth is:
    what they show is where across the face, the depth error being hidden behind the cylinder otherwise;
  * **a red bar**: the lock's force on the key from its tip, `FORCE_M_PER_N` long a newton.

Viewer only: nothing is allocated until a viewer asks for it.
"""
from __future__ import annotations

import math

import torch

from . import keylock, leg_cfg
from .debug_vis import arrow_pose
from .leg_vis import _frame_quat

DOOR_SIZE_M = (0.45, 0.45, 0.008)
#: The cam lock's bezel round the cylinder (`keylock`'s note: 27.5 mm across).
BEZEL_R_M = 0.01375
KEYWAY_PROUD_M = 0.0004
MARK_PROUD_M = 0.0006
ESTIMATE_R_M = 0.0012
REFERENCE_R_M = 0.0008
FORCE_M_PER_N = 0.004
FORCE_R_M = 0.0006
HIDDEN = 1.0e-6

DOOR, CYLINDER, HOLE, SLOT, ESTIMATE, REFERENCE, FORCE = range(7)
KEY_BLOCK, KEY_ROUND = 7, 12                     # + colour
GREY, ORANGE, BLUE, GREEN, RED = range(5)
COLOURS = ((0.55, 0.55, 0.58), (1.0, 0.5, 0.05), (0.15, 0.4, 0.95), (0.1, 0.75, 0.25), (0.9, 0.1, 0.1))
MARKERS_PER_ENV = 11   # door, cylinder, hole, slot, shaft, ward, bar, platform, estimate, reference, force


def marker_cfg(prim_path: str):
    import isaaclab.sim as sim_utils
    from isaaclab.markers import VisualizationMarkersCfg

    def material(colour):
        return sim_utils.PreviewSurfaceCfg(diffuse_color=colour)

    def block(colour):
        return sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=material(colour))

    def round_z(colour):
        return sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=material(colour))

    def sphere(colour):
        return sim_utils.SphereCfg(radius=1.0, visual_material=material(colour))

    markers = {
        "door": block((0.85, 0.82, 0.76)),
        "cylinder": round_z((0.72, 0.73, 0.76)),
        "hole": round_z((0.03, 0.03, 0.03)),
        "slot": block((0.03, 0.03, 0.03)),
        "estimate": sphere((1.0, 0.9, 0.0)),
        "reference": sphere((1.0, 0.0, 1.0)),
        "force": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="X", visual_material=material((0.9, 0.1, 0.1))),
    }
    for k, colour in enumerate(COLOURS):
        markers[f"key_block_{k}"] = block(colour)
    for k, colour in enumerate(COLOURS):
        markers[f"key_round_{k}"] = round_z(colour)
    return VisualizationMarkersCfg(prim_path=prim_path, markers=markers)


def _ward_frame(x: torch.Tensor, y: torch.Tensor, z: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """The wards' direction and its normal in the plane of a frame whose x is the bar (u) and y is -t (Link6's, and the
    lock's drawn frame): `keylock.WARD_ANGLE_RAD` is atan2(t, u)."""
    c, s = math.cos(keylock.WARD_ANGLE_RAD), math.sin(keylock.WARD_ANGLE_RAD)
    wx = c * x - s * y
    return wx, torch.cross(z, wx, dim=-1)


class InsertMarkers:
    def __init__(self, prim_path: str, num_envs: int, device: str) -> None:
        from isaaclab.markers import VisualizationMarkers

        self.markers = VisualizationMarkers(marker_cfg(prim_path))
        self.num_envs, self.device = num_envs, device

    def update(self, env) -> None:
        n, dev = self.num_envs, self.device
        lk, layer = env._lock, env._layer_ins
        origins = env.scene.env_origins
        t = torch.zeros(n, MARKERS_PER_ENV, 3, device=dev)
        q = torch.zeros(n, MARKERS_PER_ENV, 4, device=dev)
        q[..., 0] = 1.0
        s = torch.full((n, MARKERS_PER_ENV, 3), HIDDEN, device=dev)
        idx = torch.zeros(n, MARKERS_PER_ENV, dtype=torch.long, device=dev)
        code = lambda value: torch.full((n,), value, dtype=torch.long, device=dev)
        vec = lambda *v: torch.tensor(v, device=dev).expand(n, len(v))

        # The lock, in a right-handed drawn frame (u, -t, axis) as Link6's.
        a, u = lk.axis, lk.e_u
        y_l = -lk.e_t
        quat_l = _frame_quat(u, y_l, a)
        o = lk.origin + origins
        face = keylock.LOCK_FACE_OFF_DOOR_M
        t[:, 0] = o + (face + 0.5 * DOOR_SIZE_M[2]) * a
        q[:, 0], s[:, 0], idx[:, 0] = quat_l, vec(*DOOR_SIZE_M), code(DOOR)
        t[:, 1] = o + 0.5 * face * a
        q[:, 1], s[:, 1], idx[:, 1] = quat_l, vec(BEZEL_R_M, BEZEL_R_M, face), code(CYLINDER)
        hole_r = lk.hole_r
        t[:, 2] = o - 0.5 * KEYWAY_PROUD_M * a
        q[:, 2], idx[:, 2] = quat_l, code(HOLE)
        s[:, 2] = torch.stack((hole_r, hole_r, torch.full_like(hole_r, KEYWAY_PROUD_M)), -1)
        wx, wy = _ward_frame(u, y_l, a)
        t[:, 3] = t[:, 2]
        q[:, 3], idx[:, 3] = _frame_quat(wx, wy, a), code(SLOT)
        s[:, 3] = torch.stack((2 * (keylock.WARD_HALF_LEN_M + lk.play), 2 * (keylock.WARD_HALF_WIDTH_M + lk.play),
                               torch.full_like(hole_r, KEYWAY_PROUD_M)), -1)

        # The key in the jaws: Link6's frame, the key slid back by the grip's slip.
        rot6 = env._robot.data.body_quat_w[:, env._link6]
        from .keyhold import quat_matrix
        m6 = quat_matrix(rot6)
        x6, y6, z6 = m6[:, :, 0], m6[:, :, 1], m6[:, :, 2]
        p6 = env._robot.data.body_pos_w[:, env._link6]
        quat6 = _frame_quat(x6, y6, z6)
        back = keylock.KEY_BACK_LINK6_Z_M - lk.slip_m
        at = lambda bx, by, bz: p6 + bx.unsqueeze(-1) * x6 + by.unsqueeze(-1) * y6 + bz.unsqueeze(-1) * z6
        zero = torch.zeros_like(back)
        length = keylock.KEY_LENGTH_M
        started = env._layer_started
        phase = layer.phase
        colour = torch.where(phase == keylock.SEARCH, code(ORANGE),
                             torch.where((phase == keylock.SEAT) | (phase == keylock.HOLD) | (phase == keylock.DONE),
                                         code(BLUE), code(GREY)))
        colour = torch.where(started, colour, code(GREY))
        colour = torch.where(lk.seated, code(GREEN), colour)
        colour = torch.where(lk.lost | env._damaged, code(RED), colour)
        t[:, 4] = at(zero, zero, back + 0.5 * length)
        q[:, 4], idx[:, 4] = quat6, KEY_ROUND + colour
        s[:, 4] = vec(keylock.SHAFT_R_M, keylock.SHAFT_R_M, length)
        kx, ky = _ward_frame(x6, y6, z6)
        t[:, 5] = t[:, 4]
        q[:, 5], idx[:, 5] = _frame_quat(kx, ky, z6), KEY_BLOCK + colour
        s[:, 5] = vec(2 * keylock.WARD_HALF_LEN_M, 2 * keylock.WARD_HALF_WIDTH_M, length)
        # The bar and the platform (`leg_cfg.KEY_*_BOX`: along the key, off the door, up its top side), off the door
        # counted from the bar's front face, where the shaft leaves it; up the top side is -y in Link6.
        front = leg_cfg.KEY_BAR_BOX[1][0]
        for slot, box in ((6, leg_cfg.KEY_BAR_BOX), (7, leg_cfg.KEY_PAD_BOX)):
            (a0, a1), (b0, b1), (c0, c1) = box
            t[:, slot] = at(torch.full_like(back, 0.5 * (a0 + a1)), torch.full_like(back, -0.5 * (c0 + c1)),
                            back - (0.5 * (b0 + b1) - front))
            q[:, slot], idx[:, slot] = quat6, KEY_BLOCK + colour
            s[:, slot] = vec(a1 - a0, c1 - c0, b1 - b0)

        # The estimate and the layer's reference, across the true face, just in front of it.
        def on_face(p_env: torch.Tensor) -> torch.Tensor:
            lq = lk.to_lock(p_env)
            lq[:, 2] = -MARK_PROUD_M
            return lk.origin + lq[:, :1] * lk.e_u + lq[:, 1:2] * lk.e_t + lq[:, 2:3] * a + origins

        t[:, 8], idx[:, 8], s[:, 8] = on_face(env._est_o), code(ESTIMATE), vec(ESTIMATE_R_M, ESTIMATE_R_M, ESTIMATE_R_M)
        ref = env._est_point(layer.ref_depth, layer.ref_lateral)
        t[:, 9], idx[:, 9] = on_face(ref), code(REFERENCE)
        s[:, 9] = vec(REFERENCE_R_M, REFERENCE_R_M, REFERENCE_R_M) * started.float().unsqueeze(-1).clamp(min=HIDDEN)

        # The lock's force on the key, from its tip.
        tip = at(zero, zero, back + length)
        f = lk.force_w * FORCE_M_PER_N
        quat_f, flen = arrow_pose(f)
        t[:, 10] = tip + 0.5 * f
        q[:, 10], idx[:, 10] = quat_f, code(FORCE)
        shown = (lk.force_w.norm(dim=-1) > 0.2).float()
        s[:, 10] = torch.stack((flen, torch.full_like(flen, FORCE_R_M), torch.full_like(flen, FORCE_R_M)), -1)
        s[:, 10] = s[:, 10] * shown.unsqueeze(-1).clamp(min=HIDDEN)

        s[~env._ins_active] = HIDDEN
        self.markers.visualize(translations=t.view(-1, 3), orientations=q.view(-1, 4), scales=s.view(-1, 3),
                               marker_indices=idx.view(-1))

    def set_visibility(self, visible: bool) -> None:
        self.markers.set_visibility(visible)

"""Lukas's key holster beside the D1's base: where it is, what it holds the key by, and the arm drawing the key out of
it, as numbers. numpy only, except the registration (`register`, `trimesh`).

Lukas designed the holster in Fusion 360 on the D1 CAD and exported it on 2026-10-23 as three STL files, in metres, in
the CAD's own frame (the D1-550 STEP's, as `demos/cup/pick_demo/bracket.py` has it). The first export held the lever
35 degrees above level; this module found the dog's grip could not draw it out of that (below), and Lukas re-exported
it 6 degrees steeper the same day, the holster's shape round the key unchanged. `assets/holster/` is the second.

* `d1_with_holster.stl` -- the arm folded at rest, its base, and the holster (one body);
* `d1_with_lever.stl` -- the same arm with the key (two bodies: the bar with its shaft, and the platform);
* `d1_with_holster_and_lever.stl` -- both.

`register` splits them (a body in one export is the same triangles at the same place in another), finds the key's pose
by fitting the key's own bar (`assets/combinerbox_lever_v3.stl`) to the export's, and `main --write` saves the holster's
mesh in the D1's `base_link` and the key's pose there (`assets/holster/`). Everything about the key here is in the key
frame the simulation's key is drawn in (`asset.key_mesh_points`): origin on the shaft's axis at the bar's back face, x
off the back face toward the platform's side (`geometry.py`'s n), y along the lever from the shaft (u), z up the
platform's side (t).

**The CAD frame in `base_link`** (`CAD_TO_D1_BASE`): x_base = y_cad, y_base = -x_cad, z_base = z_cad + 5 mm. The base's
flange has its four mounting holes 130 mm apart along the CAD's y and 58 mm across, and `base_link`'s rail runs along
its x (`d1_arm/measured_base.py`: 130 along, 58 across); the flange's underside, the face on the rails, is 5 mm under
the CAD's origin. The 180-degree ambiguity left by the holes is settled by the folded arm: the CAD's spans y -0.283 to
0.141 m, and `d1.urdf` at the rest pose spans x -0.285 to 0.142. So the holster is on the robot's **right**, beside
the base.

**The key in it** is v3 **mirrored**, the key the simulation's insertion and turn use (`geometry.KEY_MIRRORED`): fitted
by a rotation, the mirrored key's bar lies on the export's to 0.02 mm median; v3 as drawn fits it only reflected
(`register` tries both). (The first version of this module fitted v3 as drawn with a reflection, a determinant of -1, and called
the key "as drawn"; that was wrong.) The holster is chiral: its front hook (`HOLSTER_BOXES`, `front_hook`) is where v3
as drawn would have its platform. In the holster the key's shaft points out and down from the robot's side, its platform faces in, toward the base, and
the lever runs back and down from the shaft at 41 degrees.

**What holds it.** A sleeve round the bar's far 78 mm: a cup round the bar's rounded end, two rails behind its back face,
and near the open end a C-section (the back wall, lips over the bar's top and under its bottom, a hook round its front
lower edge). The inner faces stand 0.06-0.34 mm off v3's bar. Nothing else touches the key, so it slides one way only:
**along the lever, toward the shaft** (`SLIDE_OUT_KEY`; in `base_link` forward 0.76, up 0.65: 41 degrees above level).
Standing level, gravity holds it down in the cup.

**The draw** (`draw_points`, `check`). The jaws take the bar's root round the shaft as the dog's insertion holds the
key (`robot/insert_plan.KEY_IN_JAWS_LINK6`: the shaft on the gripper's axis, the fingertips at the bar's back face,
the jaws across its 20 mm), so the key comes out of the holster in the hand as the insertion wants it, with the hand
rolled `GRIP_ROLL` (which turns it 180 degrees about the shaft from the insertion's own grip: below). The hand comes in out along the lever's axis past its root, slides onto the bar, closes, draws the
key out along the axis by `SLIDE_M`, and holds it there. The arm is folded tight here: the grip has the wrist pitch
near Joint1's axis, so drawing the hand out swings Joint1 round. At 35 degrees (the first export) the grip held the
elbow at 92% of its range and Joint1 reached its limit 70-75 mm out, where the bar's far end needs 75 mm to leave the
holster; gripped 25-30 mm along the bar it drew 90-105 mm. Tilting the holster about its cup (the CPU model,
2026-10-23) traded the elbow for the shoulder: 4-8 degrees steeper drew 95-115 mm from the root with every joint at
88-92% of its range; 12 or more and the shoulder (Joint2) is at its limit and the grip does not solve.

This is the design, not a measurement: where the holster is printed and bolted, and where the arm's joints really sit,
are what put the bar between the jaws.

    python -m demos.combiner_key.holster ~/Downloads/d1_with_holster.stl ~/Downloads/d1_with_lever.stl \\
        ~/Downloads/d1_with_holster_and_lever.stl            # report only
    ... --write                                             # and write assets/holster/
    python -m demos.combiner_key.holster --check            # the draw on the CPU arm model
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = Path(__file__).resolve().parent / "assets" / "holster"
HOLSTER_MESH_PATH = ASSET_DIR / "holster_d1_base.stl"
REGISTRATION_PATH = ASSET_DIR / "registration.json"
KEY_STL_PATH = Path(__file__).resolve().parent / "assets" / "combinerbox_lever_v3.stl"

#: The CAD frame to the D1's `base_link` (module docstring).
CAD_TO_D1_BASE = np.array([[0.0, 1.0, 0.0, 0.0],
                           [-1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.005],
                           [0.0, 0.0, 0.0, 1.0]])

# --- the parts, in the key frame (metres; module docstring) ----------------------------------------------------------

#: The holster as boxes (lo, hi), from its sections and rays cast from the bar's middle (2026-10-23). The C-section's
#: y and the cup's are their flat parts; the cup's curved inside, round the bar's rounded end, is left out (the bar's
#: collider below ends square, 0.2 mm short of the cup's end face).
HOLSTER_BOXES = {
    "rail_bottom": ((-0.00300, 0.08500, -0.01270), (-0.00034, 0.15100, -0.00730)),
    "rail_top": ((-0.00300, 0.08500, 0.00730), (-0.00034, 0.15100, 0.01270)),
    "back_wall": ((-0.00284, 0.08500, -0.01270), (-0.00034, 0.11350, 0.01270)),
    "top_lip": ((-0.00284, 0.08500, 0.01020), (0.01755, 0.11350, 0.01270)),
    "bottom_lip": ((-0.00284, 0.08500, -0.01270), (0.01755, 0.11350, -0.01020)),
    "front_hook": ((0.01506, 0.08500, -0.01270), (0.01755, 0.11350, -0.00512)),
    "cup_end": ((-0.00310, 0.16020, -0.01270), (0.01470, 0.16270, 0.01270)),
    "cup_bottom": ((-0.00310, 0.15100, -0.01270), (0.01470, 0.16020, -0.01020)),
    "cup_top": ((-0.00310, 0.15100, 0.01020), (0.01470, 0.16020, 0.01270)),
}
#: The key's colliders for the holster: its bar (square-ended; the real one is rounded at the far end), the platform
#: beside it, and the shaft and its wards (square, as `geometry.KEY_SHAFT_BOX` and `KEY_WARD_BOX`).
KEY_BOXES = {
    "bar": ((0.000, -0.015, -0.010), (0.015, 0.160, 0.010)),
    "platform": ((0.015, 0.048, 0.0075), (0.050, 0.148, 0.010)),
    "shaft": ((-0.010, -0.005, -0.005), (0.000, 0.005, 0.005)),
    "wards": ((-0.010, -0.0075, -0.00175), (0.000, 0.0075, 0.00175)),
}
#: The lever's way out of the holster, in the key frame: toward the shaft.
SLIDE_OUT_KEY = (0.0, -1.0, 0.0)

# --- the draw ------------------------------------------------------------------------------------------------------

#: Where along the bar the jaws close, from the shaft: at it, the dog's insertion grip
#: (`robot/insert_plan.KEY_IN_JAWS_LINK6`; the simulated insertion's is 25 mm along, `geometry.KEY_GRASP_U_M`).
GRIP_ALONG_M = 0.0
#: The jaw centre (the IK's controlled point, `grasp.JAW_CENTRE_LINK6`) along Link6's z.
JAW_CENTRE_Z_M = 0.1051
#: Which way round the hand holds the bar: `decoupled_arm.hand_rotation(approach=-n, bar=u, 0, GRIP_ROLL)`. The
#: insertion's grip is roll -1 (`geometry.GRASP_ROLL`: Link6's y up the platform's side); +1 is it turned 180 degrees
#: about the shaft. Which one the draw takes is `check`'s and the module docstring's.
GRIP_ROLL = +1
#: The key in the jaws, its origin in Link6: `GRIP_ALONG_M` along the bar (Link6's x is `GRIP_ROLL` times u), 1 mm
#: past the fingertips, the jaws across the bar's top and bottom.
KEY_ORIGIN_LINK6 = (-GRIP_ROLL * GRIP_ALONG_M, 0.0, 0.1266)
#: The IK's joint-limit factor through the draw: the control loop's own (`position_only.workspace.SOFT_LIMIT_FACTOR`).
IK_SOFT = 0.9
#: Where the hand waits before sliding onto the bar, out along the lever's axis: the fingers (26 mm wide) 17 mm past
#: the bar's end, which is 15 mm past the shaft.
PRE_OUT_M = 0.045
#: How far the key is drawn along its axis: the bar's far end is 75 mm inside the holster's open end.
SLIDE_M = 0.10
#: The jaw travel each side: open to take the bar (41.2 mm gap for its 20 mm), and holding it (21.2 mm), as the key
#: demo's (`env.JAW_OPEN_M`, `env.JAW_HOLD_M`).
JAW_OPEN_M = 0.012
JAW_HOLD_M = 0.002


def _sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# --- the registration (trimesh) ----------------------------------------------------------------------------------

def _parts(path):
    import trimesh

    return trimesh.load(str(path), force="mesh").split(only_watertight=False)


def _key(part):
    return len(part.faces), tuple(np.round(part.bounds.ravel(), 5))


def split_export(holster_stl, lever_stl, both_stl):
    """(holster parts, key parts, arm parts): the three files share a frame and their common bodies' triangles."""
    with_holster, with_lever = _parts(holster_stl), _parts(lever_stl)
    lever_keys = {_key(p) for p in with_lever}
    holster_keys = {_key(p) for p in with_holster}
    holster = [p for p in with_holster if _key(p) not in lever_keys]
    key = [p for p in _parts(both_stl) if _key(p) not in holster_keys]
    arm = [p for p in with_holster if _key(p) in lever_keys]
    if len(holster) != 1 or len(key) != 2:
        raise ValueError(f"expected one holster body and two key bodies, found {len(holster)} and {len(key)}")
    return holster, sorted(key, key=lambda p: -len(p.faces)), arm


def flange_holes(arm) -> np.ndarray:
    """The base flange's four mounting holes (x, y) in the CAD: the circles in its section 1.5 mm above its underside."""
    flange = min((p for p in arm if p.bounds[1, 2] < 0.0), key=lambda p: p.bounds[0, 2])
    z = flange.bounds[0, 2] + 0.0015
    section = flange.section(plane_origin=(0.0, 0.0, z), plane_normal=(0.0, 0.0, 1.0))
    holes = []
    for loop in section.discrete:
        span = np.ptp(loop[:, :2], axis=0)
        if span.max() < 0.008:
            holes.append(loop[:, :2].mean(axis=0))
    return np.array(sorted(holes, key=lambda h: (round(h[1], 3), h[0])))


def _key_mesh(mirrored: bool):
    """v3 in the key frame (module docstring), mirrored as the simulation draws it (`asset.key_mesh_points`) or not; and
    its bar with the shaft alone."""
    import trimesh

    from .geometry import STL_AXIS_YZ, STL_BACK_FACE_X

    key = trimesh.load(str(KEY_STL_PATH), force="mesh")
    flip = -1.0 if mirrored else 1.0
    move = np.diag([1.0, flip, 1.0, 1.0])
    move[:3, 3] = (-STL_BACK_FACE_X, -flip * STL_AXIS_YZ[0], -STL_AXIS_YZ[1])
    key.apply_transform(move)
    if mirrored:
        key.invert()
    return key, max(key.split(only_watertight=False), key=lambda p: len(p.faces))


def fit_key(key_cad, mirrored: bool = True) -> tuple[np.ndarray, np.ndarray]:
    """v3's bar (with its shaft), mirrored or not, fitted to the export's by a rotation: (4x4 key frame -> CAD,
    nearest-surface distances of 4000 of the fitted bar's surface points, m). The whole key (its platform too) is first
    put on both of the export's key bodies (`trimesh.registration.mesh_other`: principal axes, their flips, ICP, no
    reflection), which fixes the place along the bar; then the bar alone is fitted from there. The bar alone is nearly
    symmetric and its lattice has local fits every few centimetres along it (fits from its principal axes alone stopped
    52 mm along, 2026-10-23)."""
    import trimesh
    from trimesh.registration import icp, mesh_other

    key, bar = _key_mesh(mirrored)
    np.random.seed(0)
    initial, _ = mesh_other(key, trimesh.util.concatenate(key_cad), samples=2000, scale=False, icp_first=10,
                            icp_final=100, reflection=False)
    source = trimesh.sample.sample_surface(bar, 3000, seed=1)[0]
    matrix, _, _ = icp(source, key_cad[0], initial=initial, threshold=1e-9, max_iterations=200, reflection=False,
                       scale=False)
    if np.linalg.det(matrix[:3, :3]) < 0.0:
        raise ValueError("the key's fit came out a reflection")
    fitted = bar.copy()
    fitted.apply_transform(matrix)
    distances = trimesh.proximity.closest_point(key_cad[0], trimesh.sample.sample_surface(fitted, 4000, seed=2)[0])[1]
    return matrix, distances


def register(holster_stl, lever_stl, both_stl) -> dict:
    """The holster, the key's pose (the mirrored key's fit), and both hands' fits for the record."""
    holster, key, arm = split_export(holster_stl, lever_stl, both_stl)
    key_to_cad, distances = fit_key(key, mirrored=True)
    try:
        as_drawn = fit_key(key, mirrored=False)[1] * 1000.0
    except ValueError:
        as_drawn = None
    holes = flange_holes(arm)
    lows = np.array([p.bounds[0] for p in arm])
    arm_y = (float(min(p.bounds[0, 1] for p in arm)), float(max(p.bounds[1, 1] for p in arm)))
    return {"holster": holster[0], "key_to_cad": key_to_cad, "fit_mm": distances * 1000.0,
            "as_drawn_fit_mm": as_drawn, "holes_cad_m": holes,
            "base_underside_cad_m": float(lows[:, 2].min()), "arm_y_cad_m": arm_y,
            "inputs": {Path(p).name: {"sha256": _sha256(p), "bytes": Path(p).stat().st_size}
                       for p in (holster_stl, lever_stl, both_stl)}}


def _fit_stats(mm) -> dict:
    return {"median": round(float(np.median(mm)), 3), "p95": round(float(np.percentile(mm, 95)), 3),
            "max": round(float(mm.max()), 3)}


def write(result) -> dict:
    """The holster's mesh in `base_link` and the registration, to `ASSET_DIR`."""
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    mesh = result["holster"].copy()
    mesh.apply_transform(CAD_TO_D1_BASE)
    mesh.export(str(HOLSTER_MESH_PATH))
    key_in_base = CAD_TO_D1_BASE @ result["key_to_cad"]
    record = {
        "method": ("cad: Lukas's holster (Fusion 360, 2026-10-23) on the D1 CAD; the CAD frame to base_link by the base "
                   "flange's holes and underside and the folded arm's extent; the key by the mirrored v3's bar fitted to "
                   "the export's by a rotation (demos/combiner_key/holster.py)"),
        "measured": False,
        "cad_to_d1_base": CAD_TO_D1_BASE.tolist(),
        "key_frame": "asset.key_mesh_points: origin on the shaft's axis at the bar's back face; x n, y u, z t",
        "key_in_d1_base": np.round(key_in_base, 7).tolist(),
        "key_mirrored": True,
        "bar_fit_mm": _fit_stats(result["fit_mm"]),
        "as_drawn_bar_fit_mm": (_fit_stats(result["as_drawn_fit_mm"]) if result["as_drawn_fit_mm"] is not None
                                else "fits only as a reflection"),
        "inputs": result["inputs"],
        "holster_mesh": {"path": str(HOLSTER_MESH_PATH.relative_to(ROOT)), "sha256": _sha256(HOLSTER_MESH_PATH)},
    }
    REGISTRATION_PATH.write_text(json.dumps(record, indent=2) + "\n")
    return record


def report(result) -> list[str]:
    holes = result["holes_cad_m"]
    span = np.ptp(holes, axis=0) * 1000.0 if len(holes) else np.zeros(2)
    fit = _fit_stats(result["fit_mm"])
    other = (_fit_stats(result["as_drawn_fit_mm"]) if result["as_drawn_fit_mm"] is not None
             else "no rotation; its best fit is a reflection")
    key_in_base = CAD_TO_D1_BASE @ result["key_to_cad"]
    slide = key_in_base[:3, :3] @ np.asarray(SLIDE_OUT_KEY)
    lo, hi = (CAD_TO_D1_BASE[:3, :3] @ result["holster"].bounds.T).T + CAD_TO_D1_BASE[:3, 3]
    return [
        f"flange holes: {len(holes)}, {span[0]:.1f} mm across the CAD's x and {span[1]:.1f} along its y "
        f"(base_link: 58 across, 130 along its x); centred at {np.round(holes.mean(axis=0) * 1000, 2).tolist()} mm",
        f"base underside at CAD z {result['base_underside_cad_m'] * 1000:.1f} mm (CAD_TO_D1_BASE: -5.0)",
        f"the folded arm along the CAD's y: {result['arm_y_cad_m'][0]:.3f} to {result['arm_y_cad_m'][1]:.3f} m "
        f"(d1.urdf at rest along base x: -0.285 to 0.142)",
        f"the mirrored key's bar on the export's (a rotation): {fit} mm",
        f"v3 as drawn's bar on it (a rotation): {other} mm",
        f"holster in base_link: x {lo[0]:.4f}..{hi[0]:.4f}, y {min(lo[1], hi[1]):.4f}..{max(lo[1], hi[1]):.4f}, "
        f"z {lo[2]:.4f}..{hi[2]:.4f} m",
        f"the key's way out in base_link: {np.round(slide, 3).tolist()}, "
        f"{math.degrees(math.asin(float(slide[2]))):.1f} deg above level",
    ]


# --- the saved registration (no trimesh) -------------------------------------------------------------------------

def load() -> dict:
    return json.loads(REGISTRATION_PATH.read_text())


def key_in_d1_base(record: dict | None = None) -> np.ndarray:
    """4x4: the key frame (module docstring) to the D1's `base_link`, with the key in the holster."""
    return np.asarray((record or load())["key_in_d1_base"], dtype=float)


def key_axes(key: np.ndarray) -> dict:
    """The holstered key's origin, `u` (along the lever from the shaft), `n` (off the back face toward the platform's
    side), `t` (up the platform's side) and its way out, in whatever frame `key` (the key frame there) is."""
    rot = key[:3, :3]
    return {"origin": key[:3, 3].copy(), "u": rot[:, 1], "n": rot[:, 0], "t": rot[:, 2],
            "out": rot @ np.asarray(SLIDE_OUT_KEY)}


def grip_rotation(axes: dict, roll: int = GRIP_ROLL) -> np.ndarray:
    """Link6's axes holding the key at its root, as the control loop builds them from the script's approach and bar:
    `decoupled_arm.hand_rotation(-n, u, 0, roll)` (the approach along the shaft, the jaws across the bar's height)."""
    z = -axes["n"] / np.linalg.norm(axes["n"])
    y = roll * np.cross(z, axes["u"])
    y /= np.linalg.norm(y)
    return np.column_stack([np.cross(y, z), y, z])


def draw_points(key: np.ndarray, roll: int = GRIP_ROLL) -> dict:
    """The jaw centre for each step of the draw, in `key`'s frame, with the hand's rotation: `grip` on the bar
    `GRIP_ALONG_M` from the shaft, `pre` out along the axis, `out` drawn by `SLIDE_M` (the bar's far end 25 mm past the
    holster's open end)."""
    axes = key_axes(key)
    rot = grip_rotation(axes, roll)
    grip = axes["origin"] + GRIP_ALONG_M * axes["u"] + (KEY_ORIGIN_LINK6[2] - JAW_CENTRE_Z_M) * axes["n"]
    out = grip + SLIDE_M * axes["out"]
    return {"rot": rot, "pre": grip + PRE_OUT_M * axes["out"], "grip": grip, "out": out}


def boxes_in(key: np.ndarray, boxes: dict) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """The boxes (given in the key frame) as (centre, axes as columns, half sizes) in `key`'s frame."""
    out = []
    for lo, hi in boxes.values():
        lo, hi = np.asarray(lo), np.asarray(hi)
        out.append((key[:3, :3] @ ((lo + hi) / 2) + key[:3, 3], key[:3, :3], (hi - lo) / 2))
    return out


def box_distance(points: np.ndarray, boxes) -> np.ndarray:
    """Each point's distance outside the nearest box (negative: how deep inside it)."""
    best = np.full(len(points), np.inf)
    for centre, axes, half in boxes:
        local = np.abs((points - centre) @ axes) - half
        outside = np.linalg.norm(np.maximum(local, 0.0), axis=1)
        inside = np.minimum(local.max(axis=1), 0.0)
        best = np.minimum(best, outside + inside)
    return best


# --- the draw on the CPU arm model --------------------------------------------------------------------------------

def _arm_points(joints, q, gripper_m: float) -> dict:
    """Each D1 link's mesh vertices (decimated) at `q`, in the Go2 base frame, the jaws at `gripper_m` travel."""
    from position_only.workspace import forward, read_stl

    frames, _, _ = forward(joints, np.asarray(q, dtype=float).reshape(1, 6),
                           gripper=np.array([[gripper_m, -gripper_m]]))
    out = {}
    for link, (rot, pos) in frames.items():
        mesh = ROOT / "d1_arm" / "meshes" / ("base_link_measured.STL" if link == "base_link" else f"{link}.STL")
        if not mesh.exists():
            continue
        vertices = _mesh_cache.setdefault(link, read_stl(mesh)[::3])
        out[link] = vertices @ rot[0].T + pos[0]
    return out


_mesh_cache: dict = {}
#: The hand: what holds the key, so its distance to the key is not a collision.
HAND_LINKS = ("Link6", "Link7_1", "Link7_2")


def check(roll: int = GRIP_ROLL, soft: float = IK_SOFT, steps: int = 11) -> dict:
    """The draw on the CPU arm model, the robot standing level: IK for each point (seeded from the folded rest, then
    warm-started along the moves as the control loop runs), and along the way the trunk proxy, the gravity hold, the
    joints' use of their range, the hand and arm against the holster, and the key against the arm's base as it
    slides."""
    import d1_hardware
    import d1_ik
    from demos.cup.pick_demo.grasp import _seeds, arm_clear, proxy_margin
    from position_only.workspace import EFFORT_LIMIT_NM, MOUNT_B, gravity_torques, load_urdf

    joints, links = load_urdf()
    key_go2 = key_in_d1_base().copy()
    key_go2[:3, 3] += MOUNT_B
    pts = draw_points(key_go2, roll)
    rot = pts["rot"]
    offset = (0.0, 0.0, JAW_CENTRE_Z_M)
    rest = np.asarray(d1_ik.from_servo_deg(d1_hardware.REST_SERVO_DEG)[0], dtype=float)
    lows, highs = d1_ik.joint_limits(joints, 1.0)
    holster = boxes_in(key_go2, HOLSTER_BOXES)
    moves = [("pre", pts["pre"], pts["pre"], JAW_OPEN_M), ("on", pts["pre"], pts["grip"], JAW_OPEN_M),
             ("out", pts["grip"], pts["out"], JAW_HOLD_M)]
    q = None
    rows = []
    for name, start, end, jaw in moves:
        for f in np.linspace(0.0, 1.0, steps if name != "pre" else 1):
            point = start + f * (end - start)
            seeds = [q] if q is not None else [rest] + _seeds(point)
            result = None
            for seed in seeds:
                r = d1_ik.solve(joints, point, rot, q0=seed, body="Link6", offset=offset, max_iterations=400, soft=soft)
                if r.converged and bool(arm_clear(joints, r.q)[0]):
                    result = r
                    break
            if result is None:
                rows.append({"move": name, "f": round(float(f), 2), "solved": False})
                return {"solved": False, "rows": rows, "points": pts}
            q = result.q
            slid = SLIDE_M * f if name == "out" else 0.0
            arm = _arm_points(joints, q, jaw)
            hand = np.vstack([arm[k] for k in HAND_LINKS if k in arm])
            links_moving = np.vstack([v for k, v in arm.items() if k not in HAND_LINKS and k != "base_link"])
            moved = key_go2.copy()
            moved[:3, 3] += slid * key_axes(key_go2)["out"]
            key_boxes = boxes_in(moved, KEY_BOXES)
            # Before the jaws close the hand must not touch the key: the open fingers either side of the bar.
            hand_to_key = (round(1000 * float(box_distance(hand, key_boxes).min()), 1) if name in ("pre", "on")
                           else None)
            use = np.abs(gravity_torques(joints, links, q[None]))[0] / EFFORT_LIMIT_NM
            span = np.where(q >= 0, q / highs, q / lows)
            rows.append({"move": name, "f": round(float(f), 2), "solved": True, "q": np.round(q, 3).tolist(),
                         "range_use_max": round(float(span.max()), 3), "worst_joint": int(np.argmax(span)) + 1,
                         "gravity_use_max": round(float(use.max()), 3),
                         "trunk_proxy_margin_m": round(proxy_margin(joints, q), 3),
                         "hand_to_holster_mm": round(1000 * float(box_distance(hand, holster).min()), 1),
                         "open_hand_to_key_mm": hand_to_key,
                         "arm_to_holster_mm": round(1000 * float(box_distance(links_moving, holster).min()), 1),
                         "key_to_arm_base_mm": round(1000 * float(box_distance(arm["base_link"], key_boxes).min()), 1),
                         "key_to_arm_links_mm": round(1000 * float(box_distance(links_moving, key_boxes).min()), 1)})
    # The unfold (`unfold_stages`): from the folded rest to `pre` in joint space, as the D1's planner moves it (each
    # joint at its own speed ceiling), against the holster, the key in it and the trunk proxy.
    q_pre = np.asarray(rows[0]["q"])
    path = np.concatenate([d1_ik.traversal_configs(a, b) for a, b in
                           zip([rest, *unfold_stages(rest, q_pre)[:-1]], unfold_stages(rest, q_pre))])
    key_boxes = boxes_in(key_go2, KEY_BOXES)
    unfold = []
    for qq in path:
        arm = _arm_points(joints, qq, JAW_OPEN_M)
        moving = np.vstack([v for k, v in arm.items() if k != "base_link"])
        unfold.append(min(float(box_distance(moving, holster).min()), float(box_distance(moving, key_boxes).min())))
    return {"solved": True, "rows": rows, "points": pts, "q_pre": q_pre, "rest": rest,
            "unfold_clearance_mm": round(1000 * min(unfold), 1),
            "unfold_trunk_proxy_margin_m": round(min(proxy_margin(joints, qq) for qq in path), 3)}


def joint_paced(joint_speed: float, roll: int = GRIP_ROLL, soft: float = IK_SOFT, samples: int = 41) -> dict:
    """The draw's two straight moves (`on`: pre to grip, `draw`: grip to out) timed so that the arm's fastest joint
    turns at `joint_speed` rad/s throughout, on the CPU model: {move: (seconds, u, s)}, where `s` (0 to 1 along the
    line) at the phase's progress `u` is `np.interp(u, u_table, s_table)`.

    Why: the D1's firmware plans each 10 Hz setpoint from rest (`motor_model.D1_REPLAN_VELOCITY_RETENTION`, F-045), so a
    joint asked for under ~2.5 degrees a command gets there and stops until the next one: slow moves go stop-start.
    And the draw is far from uniform in joint space -- the wrist roll turns 1.8 rad, most of it in the draw's second
    half -- so a move timed evenly along the line is slow at one end and past the joints' ~1.25 rad/s at the other."""
    import d1_ik
    from demos.cup.pick_demo.grasp import _seeds, arm_clear
    from position_only.workspace import MOUNT_B, load_urdf

    joints, _ = load_urdf()
    key = key_in_d1_base().copy()
    key[:3, 3] += MOUNT_B
    pts = draw_points(key, roll)
    offset = (0.0, 0.0, JAW_CENTRE_Z_M)
    q = None
    out = {}
    for move, a, b in (("on", pts["pre"], pts["grip"]), ("draw", pts["grip"], pts["out"])):
        s = np.linspace(0.0, 1.0, samples)
        qs = []
        for f in s:
            point = a + f * (b - a)
            # As `check` seeds it: the folded rest first, then the scripted grip's seeds; then warm-started.
            for seed in ([q] if q is not None else [_rest_q()] + _seeds(point)):
                r = d1_ik.solve(joints, point, pts["rot"], q0=seed, body="Link6", offset=offset, max_iterations=400,
                                soft=soft)
                if r.converged and bool(arm_clear(joints, r.q)[0]):
                    break
            q = r.q
            qs.append(q)
        step = np.abs(np.diff(np.asarray(qs), axis=0)).max(axis=1)
        t = np.concatenate([[0.0], np.cumsum(step / joint_speed)])
        out[move] = (float(t[-1]), t / t[-1], s)
    return out


def _rest_q() -> np.ndarray:
    import d1_hardware
    import d1_ik

    return np.asarray(d1_ik.from_servo_deg(d1_hardware.REST_SERVO_DEG)[0], dtype=float)


def unfold_stages(rest, q_pre) -> list[np.ndarray]:
    """The joint-space goals from the folded rest to the hand at `pre`: Joint1 turned first, the arm still folded, then
    the rest. Straight from rest to `pre` the D1's joint-by-joint motion dips the hand 7 mm into the trunk proxy's 3 cm
    margin; turned first, it keeps 32 mm (the CPU model, 2026-10-23)."""
    turned = np.asarray(rest, dtype=float).copy()
    turned[0] = q_pre[0]
    return [turned, np.asarray(q_pre, dtype=float)]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stl", nargs="*", help="d1_with_holster.stl d1_with_lever.stl d1_with_holster_and_lever.stl")
    parser.add_argument("--write", action="store_true", help="write the holster mesh and the registration")
    parser.add_argument("--check", action="store_true", help="the draw on the CPU arm model")
    parser.add_argument("--roll", type=int, default=GRIP_ROLL)
    parser.add_argument("--soft", type=float, default=IK_SOFT)
    args = parser.parse_args(argv)
    if args.stl:
        result = register(*args.stl)
        print("\n".join(report(result)))
        if args.write:
            record = write(result)
            print(f"wrote {REGISTRATION_PATH.relative_to(ROOT)} and {record['holster_mesh']['path']}")
    if args.check:
        out = check(args.roll, args.soft)
        for row in out["rows"]:
            print(json.dumps(row))
        if out["solved"]:
            print(json.dumps({"unfold_clearance_mm": out["unfold_clearance_mm"], "q_pre": out["q_pre"].round(4).tolist(),
                              "unfold_trunk_proxy_margin_m": out["unfold_trunk_proxy_margin_m"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

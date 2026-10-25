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

**v4** (Lukas, 2026-10-24, `register_v4`: `holster_lever.stl` and `lever_only.stl`). The holster now slides into the
rails on the Go2's expansion dock: a strut from the sleeve down to a rail slider, one print with it. The lever is the
"1 cm longer" one the draw above assumed (`LEVER_EXTRA_M`): a plain bar 15 x 20 mm, 185 mm long with its far end
rounded, no platform, the shaft 9.94 mm further out along the holster's axis than v3's, its wards 45 degrees off the
bar (v3's lie along it). Both are drawn in the 10-23 CAD frame (the sleeve's unchanged faces lie on the 10-23 holster's,
0.01 mm median), so the frames above carry over; the lever lies on the holster frame's axes to 0.06 mm and its frame,
measured, is within 0.07 mm of the 10-23 assumption. The sleeve's grip changed a little (`HOLSTER_BOXES`'s note: the
lips and the hook further off the bar, the bottom lip and hook on to the cup). Since then `HOLSTER_MESH_PATH` is v4's
holster (sleeve, strut and slider), `LEVER_V4_PATH` its lever, `KEY_BOXES` its colliders (`V3_KEY_BOXES` v3's), and the
registration keeps the 10-23 one under `replaced`.

    python -m demos.combiner_key.holster ~/Downloads/holster_lever.stl ~/Downloads/lever_only.stl [--write]

**v5** (Lukas, 2026-10-24, the same `register_v4` on `holster_with_lever2.stl` and `lever2.stl`; its
`d1_with_holster_and_lever2.stl` has the folded arm where the 10-23 exports have it): "a mouth to guide the lever
back inside, and puts the lever another cm higher so the grip can properly get it without collision". The lever is v4's
(the same 384 vertices), moved 15.000 mm out along the holster's axis (9.8 mm higher, 11.3 forward): its shaft 24.93
mm out of the holster frame. The sleeve and the strut are where v4's are; the cup is filled 15 mm (its end 0.09 mm off
the bar's), no lip over the bar's top beyond the back wall's end, and a mouth (a separate body, 11.4 cm^3) flares from
the channel 107 mm along the lever to its lip at 87 mm, four flat walls at 27-33 degrees (`MOUTH_WALLS`), so the bar's
far end, drawn past the channel, is in a funnel ~8 mm clear of it a side. The grip being 15 mm further out, the arm
reaches only ~76 mm along the axis from it at the IK's 0.9 of every joint's range (86 with Joint1 and Joint5 at 0.97,
the CPU model), where the far end needs 62 to leave the channel and 83 to leave the mouth. So the draw is `SLIDE_M`
along the axis and then `EXIT_M` tilted `EXIT_DEG` toward the mouth's front wall (`draw_points`' `exit`), through the
funnel and out past its lip; putting the key back is the same path the other way, into the funnel and down the
channel (`return` in the simulation, `insert_plan`'s `h_back`, `h_in` on the dog). V4's colliders are `V4_HOLSTER_BOXES`.

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
#: The v4 lever (Lukas, 2026-10-24, `register_v4`) in its own key frame, metres: the holster's key since then.
LEVER_V4_PATH = Path(__file__).resolve().parent / "assets" / "lever_v4.stl"

#: The CAD frame to the D1's `base_link` (module docstring).
CAD_TO_D1_BASE = np.array([[0.0, 1.0, 0.0, 0.0],
                           [-1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.005],
                           [0.0, 0.0, 0.0, 1.0]])

# --- the parts, in the key frame (metres; module docstring) ----------------------------------------------------------

#: The holster as boxes (lo, hi), from its sections and rays cast from the bar's middle (2026-10-23). The C-section's
#: y and the cup's are their flat parts; the cup's curved inside, round the bar's rounded end, is left out (the bar's
#: collider below ends square, 0.3 mm short of the cup's end face).
#: The v4 holster (2026-10-24, rays from the bar's middle against the 10-23 boxes' faces): the back wall and the rails
#: unchanged; the lips 0.15 mm further out (10.35 mm from the bar's middle, against 10.20) and the cup's sides with them;
#: the front hook 0.65 mm further out (15.83 mm off the back face, 0.83 mm off v4's bar); the cup's end 0.14 mm further;
#: the bottom lip and the hook now run on to the cup (from 113.5 mm along to 151), the top lip still stops at 113.5.
V4_HOLSTER_BOXES = {
    "rail_bottom": ((-0.00300, 0.08500, -0.01270), (-0.00034, 0.15100, -0.00730)),
    "rail_top": ((-0.00300, 0.08500, 0.00730), (-0.00034, 0.15100, 0.01270)),
    "back_wall": ((-0.00284, 0.08500, -0.01270), (-0.00034, 0.11350, 0.01270)),
    "top_lip": ((-0.00284, 0.08500, 0.01035), (0.01755, 0.11350, 0.01270)),
    "bottom_lip": ((-0.00284, 0.08500, -0.01270), (0.01755, 0.15100, -0.01035)),
    "front_hook": ((0.01583, 0.08500, -0.01270), (0.01832, 0.15100, -0.00512)),
    "cup_end": ((-0.00310, 0.16041, -0.01270), (0.01470, 0.16291, 0.01270)),
    "cup_bottom": ((-0.00310, 0.15100, -0.01270), (0.01470, 0.16041, -0.01035)),
    "cup_top": ((-0.00310, 0.15100, 0.01035), (0.01470, 0.16041, 0.01270)),
}
#: v5's (2026-10-24, rays from v5's bar's middle, n 7.5 mm, t 0, along and across it), in the same holster frame (v5's
#: lever frame is it moved 24.93 mm out, -u): the rails and back wall from 80.1 mm along (105 in the lever's frame),
#: their face 0.24 mm behind the bar; the lips (0.06 and 0.39 mm off it) and the front hook (0.83) from where the mouth
#: meets the channel, 82.1 mm (107); the back wall and top lip to 113.5 as v4's; the bottom lip, the hook and the rails
#: on to the cup's end, filled to 145.2 (170.1, the bar's end at 170.0). The cup's rounded inside is left out (the bar's
#: collider is square-ended). The mouth's walls are `mouth_plates`.
HOLSTER_BOXES = {
    "rail_bottom": ((-0.00300, 0.08007, -0.01270), (-0.00024, 0.14516, -0.00730)),
    "rail_top": ((-0.00300, 0.08007, 0.00730), (-0.00024, 0.14516, 0.01270)),
    "back_wall": ((-0.00284, 0.08007, -0.01270), (-0.00024, 0.11350, 0.01270)),
    "top_lip": ((-0.00284, 0.08207, 0.01039), (0.01755, 0.11350, 0.01270)),
    "bottom_lip": ((-0.00284, 0.08207, -0.01270), (0.01755, 0.14516, -0.01006)),
    "front_hook": ((0.01583, 0.08207, -0.01270), (0.01832, 0.14516, -0.00512)),
    "cup_end": ((-0.00310, 0.14516, -0.01270), (0.01832, 0.16291, 0.01270)),
}
#: v5's mouth, holster frame: from its lip (`MOUTH_U_M[0]`) to the channel (`[1]`), each wall's inner face a plane, its
#: normal in the (n, t) plane and its distance from the bar's middle at the lip and at the channel: lines fitted to rays
#: every 2.5 mm along it (within 0.03 mm), the four meeting the channel's faces at 107.0 mm along the lever. Their
#: slopes: back 32.8 deg, front 27.1, bottom 29.2, top 32.1.
MOUTH_U_M = (0.06207, 0.08207)
MOUTH_WALLS = {
    "back": ((-1.0, 0.0), 0.02062, 0.00773),
    "front": ((1.0, 0.0), 0.01861, 0.00835),
    "bottom": ((0.0, -1.0), 0.02125, 0.01007),
    "top": ((0.0, 1.0), 0.02291, 0.01039),
}
#: And a fifth face across its front-bottom corner, the D1 base's side (the base's mesh 0.4 mm off it): a twisted one,
#: straight in each section (within 0.01 mm), its normal turning and nearing the bar's middle toward the channel, where it
#: ends in the channel's corner. (u in the holster frame, normal's angle atan2(t, n) in degrees, distance) at four u.
MOUTH_CHAMFER = ((0.06257, -53.1, 0.01724), (0.06757, -57.1, 0.01560), (0.07257, -63.2, 0.01395),
                 (0.07757, -73.5, 0.01217))
#: The bar's middle (n, t) in the holster frame, which the walls are measured from.
BAR_MIDDLE_NT = (0.0075, 0.0)
MOUTH_WALL_M = 0.002
MOUTH_SEGMENTS = 3
#: The holstered bar's four long edges (n, t) from its middle, and how far the mouth's plates are kept off them.
BAR_EDGES_NT = ((-0.0075, -0.010), (-0.0075, 0.010), (0.0075, -0.010), (0.0075, 0.010))
MOUTH_BAR_CLEAR_M = 0.0003


def mouth_walls(u: float) -> dict:
    """The mouth's walls at `u` along the holster frame: {name: (unit normal (n, t), distance from the bar's middle)}, in
    order round the opening; the chamfer's fitted (its angle quadratic, its distance linear in u) and held off the
    channel's corner."""
    u0, u1 = MOUTH_U_M
    f = (u - u0) / (u1 - u0)
    out = {}
    for name, (normal, d_lip, d_in) in MOUTH_WALLS.items():
        out[name] = (np.asarray(normal, dtype=float), d_lip + f * (d_in - d_lip))
    us, angles, ds = (np.array(v) for v in zip(*MOUTH_CHAMFER))
    angle = math.radians(float(np.polyval(np.polyfit(us, angles, 2), u)))
    normal = np.array([math.cos(angle), math.sin(angle)])
    corner = np.array([MOUTH_WALLS["front"][2], -MOUTH_WALLS["bottom"][2]])     # the channel's front-bottom corner
    out["chamfer"] = (normal, max(float(np.polyval(np.polyfit(us, ds, 1), u)), float(corner @ normal)))
    return {k: out[k] for k in ("back", "top", "front", "chamfer", "bottom")}


def mouth_section(u: float) -> dict:
    """The opening at `u` (holster frame): {wall: (its edge's two ends, (n, t) from the bar's middle)}, each between the
    walls either side of it round the opening."""
    walls = mouth_walls(u)
    names = list(walls)
    meet = lambda a, b: np.linalg.solve(np.array([walls[a][0], walls[b][0]]), np.array([walls[a][1], walls[b][1]]))
    return {name: (meet(names[k - 1], name), meet(name, names[(k + 1) % len(names)])) for k, name in enumerate(names)}


def mouth_plates(segments: int = MOUTH_SEGMENTS) -> dict:
    """The mouth's walls as `MOUTH_WALL_M` thick plates behind their inner faces, holster frame: {name + segment:
    (centre, axes as columns (along the wall toward the channel, across it, out of the mouth), half sizes)}. Each wall
    is `segments` plates along it, each the plane through its edge of the opening at the segment's two ends
    (`mouth_section`), as long as that and as wide as the edge is at its widest, half a wall's thickness on at either
    end, so they meet at the corners and stay inside the funnel's outside, which tapers as the opening does; and held
    `MOUTH_BAR_CLEAR_M` off the holstered bar's edges. (One full-width plate a wall, and no chamfer, reached 10 mm into
    the D1 base's convex hull, as which PhysX collides the base: the kicks reset the simulation every 0.28 s.)"""
    out = {}
    u0, u1 = MOUTH_U_M
    mid = np.array([BAR_MIDDLE_NT[0], 0.0, BAR_MIDDLE_NT[1]])
    to3 = lambda nt, u: mid + np.array([nt[0], u, nt[1]])
    for seg in range(segments):
        ua, ub = u0 + seg / segments * (u1 - u0), u0 + (seg + 1) / segments * (u1 - u0)
        sa, sb = mouth_section(ua), mouth_section(ub)
        walls = mouth_walls(ua)
        for name in sa:
            corners = np.array([to3(e, ua) for e in sa[name]] + [to3(e, ub) for e in sb[name]])
            centroid = corners.mean(axis=0)
            normal = np.linalg.svd(corners - centroid)[2][2]
            outward = np.array([walls[name][0][0], 0.0, walls[name][0][1]])
            normal = normal if normal @ outward > 0.0 else -normal
            along = np.array([0.0, 1.0, 0.0]) - normal[1] * normal
            along /= np.linalg.norm(along)
            across = np.cross(normal, along)
            a = (corners - centroid) @ along
            b = (corners - centroid) @ across
            face = centroid + 0.5 * (a.min() + a.max()) * along + 0.5 * (b.min() + b.max()) * across
            # Off the bar's edges: the farthest one past the face, along its normal, and the clearance on.
            edges = [to3(np.array(c), uu) for c in BAR_EDGES_NT for uu in (ua, ub)]
            push = max(0.0, max(float((e - face) @ normal) for e in edges) + MOUTH_BAR_CLEAR_M)
            centre = face + (0.5 * MOUTH_WALL_M + push) * normal
            out[f"{name}{seg}"] = (centre, np.column_stack([along, across, normal]),
                                   np.array([0.5 * (a.max() - a.min()), 0.5 * (b.max() - b.min()) + 0.5 * MOUTH_WALL_M,
                                             0.5 * MOUTH_WALL_M]))
    return out


def _sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()



#: v3's colliders for the holster, in v3's key frame: its bar (square-ended; the real one is rounded at the far end), the
#: platform beside it, and the shaft and its wards (square, as `geometry.KEY_SHAFT_BOX` and `KEY_WARD_BOX`).
V3_KEY_BOXES = {
    "bar": ((0.000, -0.015, -0.010), (0.015, 0.160, 0.010)),
    "platform": ((0.015, 0.048, 0.0075), (0.050, 0.148, 0.010)),
    "shaft": ((-0.010, -0.005, -0.005), (0.000, 0.005, 0.005)),
    "wards": ((-0.010, -0.0075, -0.00175), (0.000, 0.0075, 0.00175)),
}
#: The holster's key since 2026-10-24: v4 (`register_v4`), in its own key frame (origin on its shaft's axis at the
#: bar's back face; x n, y u, z t as v3's). A bar 15 mm deep and 20 tall from 15 mm behind the shaft to 170 mm along it
#: (its far 10 mm rounded; the collider square-ended), and v3's platform 10 mm further out, the same from the bar's far
#: end (58-158 mm along; Lukas: "I must have forgot to export the platform in the stl files - its still there, in the
#: same position relative to the end of the lever thats in the holster", 2026-10-24); its shaft 10 mm long, 9.8 mm round with two
#: wards 15 mm across at `V4_WARD_ANGLE_DEG` (atan2(t, u)) from the bar: 45 degrees off it, where v3's lie along it. The
#: wards' box here is their extent along both axes of the face, a square-cornered stand-in. The STL's 33.8 cm^3 and
#: the platform's 8.75: `V4_MASS_KG` printed solid in PLA (v3's 43.0 cm^3 is `asset.KEY_MASS_KG`'s 53 g).
V4_KEY_BOXES = {
    "bar": ((0.000, -0.015, -0.010), (0.015, 0.170, 0.010)),
    "platform": ((0.015, 0.058, 0.0075), (0.050, 0.158, 0.010)),
    "shaft": ((-0.010, -0.005, -0.005), (0.000, 0.005, 0.005)),
    "wards": ((-0.010, -0.0053, -0.0053), (0.000, 0.0053, 0.0053)),
}
V4_WARD_ANGLE_DEG = 135.0
V4_MASS_KG = 0.0528                 # the STL's 33.8 cm^3 and the platform's 8.75, solid PLA (v3: 53 g)
#: v5's key (the holster's since 2026-10-24): v4's lever without its platform (Lukas: "No platform now"; where the model
#: had it, it passes through v5's mouth's front wall), 33.8 cm^3 of solid PLA at v4's density.
KEY_BOXES = {name: box for name, box in V4_KEY_BOXES.items() if name != "platform"}
V5_MASS_KG = round(V4_MASS_KG * 33.81 / (33.81 + 8.75), 4)
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
#: Where the hand waits before sliding onto the bar, out along the lever's axis: the fingers (26 mm wide) 37 mm past
#: the bar's end, which is 15 mm past the shaft. At 45 mm, with the lever `LEVER_EXTRA_M` longer, a finger passes the
#: key's root 2.7 mm off in the unfold; at 55, 9.8; at 65, 11.9 (the CPU model, 2026-10-23).
PRE_OUT_M = 0.065
#: How far the key is drawn along its axis, then out through the mouth (v5, module docstring): `SLIDE_M` along it (the
#: far end 13 mm past the channel, in the funnel), then `EXIT_M` at `EXIT_DEG` toward the front wall (`n`), which
#: opens at 27 degrees: the bar's front corner stays ~7 mm off it, and the far end ends 10 mm past the lip. On the CPU
#: model from the v5 grip: along the axis 76 mm at 0.9 of every joint's range; the exit then 2 mm at 0.9 and 20 at
#: `DRAW_SOFT_J1_J5` (8-14 tilted any other way). (v4: 90 mm along, the far end 75 mm inside the open end.)
SLIDE_M = 0.075
EXIT_M = 0.020
EXIT_DEG = 25.0
#: Then lifted `CLEAR_M` away from the mouth (`draw_points`' `clear`, `clear_direction`) before the arm moves on: on
#: the dog the move to the lock straight from the exit swung the key 7 mm back along the axis as it began and bumped
#: the lever's end on the mouth's lip, loosening it in the jaws (Lukas, 2026-10-25: "it still got out, but bumped the
#: key and messed up the grip"). The way: `CLEAR_OUT_DEG` out along the axis from a direction across it
#: `CLEAR_ROUND_DEG` round from the holster's open front (`n`) toward the bar's top side (`t`). On the CPU model, of 36
#: directions 9 reach 40 mm; this one keeps the key, held 6 mm along as on the dog, 5.5 mm or more off the holster and
#: 8.5 off the D1's base (straight toward the front, 0.5 mm off the base).
CLEAR_M = 0.040
CLEAR_OUT_DEG, CLEAR_ROUND_DEG = 20.0, 60.0
#: Joint1 and Joint5's share of their range through the moves out of the holster and back (`draw_soft`): the draw ends
#: where they run out (v4 on the dog: 127.7 and 86.5 deg, against 134.6 and 90).
DRAW_SOFT_J1_J5 = 0.97
#: The lever taken this much longer than v3, between the shaft and the platform, its far end still in the holster's cup:
#: the shaft and the grip sit this much further out along the holster's axis (`key_in_d1_base`, `key_boxes`). Lukas,
#: 2026-10-23, from the viewer: "the wrist motor travels through the shoulder motor. can we assume the lever is 1cm
#: longer so it grips it higher." (The simulation has the arm's self-collision off; nothing here checked link against
#: link.) The holster's own place is the export's (`holster_in_d1_base`).
LEVER_EXTRA_M = 0.010
#: v4 is that lever (2026-10-24): its shaft 9.94 mm further out along the holster's axis than v3's, its bar 10 mm
#: longer, measured off its STL (`register_v4`). `key_in_d1_base` takes the measured frame from the registration.
#: The jaw travel each side: fully open before they close on the bar (the D1's 30 mm, `grasp.GRIPPER_OPEN_M`: a 77.2 mm
#: gap round its 20 mm; Lukas, 2026-10-23: at the key demo's 12 mm, a 41.2 mm gap, "its too tight of a clearance on the
#: real dog"), and holding it (21.2 mm), as the key demo's (`env.JAW_HOLD_M`).
JAW_OPEN_M = 0.030
JAW_HOLD_M = 0.002


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


# --- the v4 design: the holster on the Go2's rails, the lever 1 cm longer (2026-10-24) -------------------------------

def register_v4(holster_lever_stl, lever_stl) -> dict:
    """Lukas's second design (Fusion 360, 2026-10-24): `holster_lever_stl` holds the holster (its sleeve, and a strut
    down to a slider for the rails on the Go2's expansion dock) with the lever in it; `lever_stl` the lever alone. Both
    in the 10-23 exports' CAD frame (the D1-550 STEP's): the sleeve's untouched faces lie on the 10-23 holster's to
    0.01 mm (median), so `CAD_TO_D1_BASE` and the holster's frame (`holster_in_d1_base`) carry over. The lever lies on
    the holster frame's axes (its faces on its planes to `fit_mm`); its frame is that one moved to its shaft's centre.
    Returns the holster's bodies (CAD), the lever's frame in CAD, its mesh in that frame, and what was measured."""
    import trimesh

    lever = trimesh.load(str(lever_stl), force="mesh")
    bodies = [b for b in trimesh.load(str(holster_lever_stl), force="mesh").split(only_watertight=False)
              if abs(b.volume) > 1e-9]                     # the export's slivers (zero-volume faces) dropped
    same = lambda a, b: len(a.vertices) == len(b.vertices) and np.allclose(np.sort(a.vertices, 0),
                                                                           np.sort(b.vertices, 0), atol=1e-6)
    holster_parts = [b for b in bodies if not same(b, lever)]
    if len(holster_parts) != len(bodies) - 1:
        raise ValueError("the lever alone is not one of the holster export's bodies")
    holster_frame_cad = np.linalg.inv(CAD_TO_D1_BASE) @ holster_in_d1_base()
    to_frame = np.linalg.inv(holster_frame_cad)
    pts = lever.vertices @ to_frame[:3, :3].T + to_frame[:3, 3]
    shaft = pts[pts[:, 0] < -0.0005]
    u0 = 0.5 * (shaft[:, 1].min() + shaft[:, 1].max())
    t0 = 0.5 * (shaft[:, 2].min() + shaft[:, 2].max())
    bar = pts[pts[:, 0] > -0.0005]
    planes = np.minimum(np.abs(bar[:, 0] - 0.0), np.abs(bar[:, 0] - 0.015))
    lever_to_cad = holster_frame_cad @ np.array([[1, 0, 0, 0], [0, 1, 0, u0], [0, 0, 1, t0], [0, 0, 0, 1.0]])
    in_frame = lever.copy()
    in_frame.apply_transform(np.linalg.inv(lever_to_cad))
    rel = shaft - np.array([0.0, u0, t0])
    radius = np.hypot(rel[:, 1], rel[:, 2])
    tips = rel[radius > 0.9 * radius.max()]
    angles = np.degrees(np.arctan2(tips[:, 2], tips[:, 1])) % 180.0
    return {"holster": holster_parts, "lever_to_cad": lever_to_cad, "lever_mesh": in_frame,
            "shaft_from_holster_frame_mm": [round(1000 * u0, 2), round(1000 * t0, 2)],
            "bar_faces_off_planes_mm": round(1000 * float(planes.max()), 3),
            "lever_bounds_m": np.round(in_frame.bounds, 5).tolist(), "lever_volume_cm3": round(lever.volume * 1e6, 2),
            "ward_angle_deg": round(float(np.median(angles)), 1), "shaft_radius_mm": round(1000 * float(radius.max()), 2),
            "holster_volumes_cm3": [round(abs(b.volume) * 1e6, 2) for b in holster_parts],
            "inputs": {Path(p).name: {"sha256": _sha256(p), "bytes": Path(p).stat().st_size}
                       for p in (holster_lever_stl, lever_stl)}}


#: What `write_v4` records as the registration's method, by design.
DESIGNS = {
    "v4": ("cad: Lukas's v4 holster and lever (Fusion 360, 2026-10-24: the holster slides into the Go2's rails on the "
           "expansion dock), in the 10-23 exports' CAD frame (the sleeve's unchanged faces on the 10-23 holster's); the "
           "lever's frame the holster's moved to its shaft (demos/combiner_key/holster.py, register_v4)"),
    "v5": ("cad: Lukas's v5 holster (Fusion 360, 2026-10-24, holster_with_lever2.stl and lever2.stl: a mouth to guide "
           "the lever back in, the lever 15 mm further out along the holster's axis, its cup filled), in the 10-23 "
           "exports' CAD frame (the sleeve and strut where v4's are; d1_with_holster_and_lever2.stl's folded arm where "
           "the 10-23 exports' is); v4's lever, its frame the holster's moved to its shaft (demos/combiner_key/"
           "holster.py, register_v4)"),
}


def write_v4(result, design: str = "v5") -> dict:
    """The holster's mesh in `base_link` (`HOLSTER_MESH_PATH`, v4's sleeve, strut and rail slider, and v5's mouth), the
    lever in its frame (`LEVER_V4_PATH`: v5's is v4's) and the registration, the one before kept under `replaced`."""
    import trimesh

    previous = load()
    mesh = trimesh.util.concatenate(result["holster"])
    mesh.apply_transform(CAD_TO_D1_BASE)
    mesh.export(str(HOLSTER_MESH_PATH))
    result["lever_mesh"].export(str(LEVER_V4_PATH))
    record = dict(previous)
    record.update({
        "method": DESIGNS[design],
        "design": design,
        "key_in_d1_base": previous["key_in_d1_base"],
        "holster_frame": "v3's key frame as the 10-23 export holds it (HOLSTER_BOXES and the holster's mesh are in it)",
        "lever": "v4's (assets/lever_v4.stl, in its own key frame)",
        "lever_in_d1_base": np.round(CAD_TO_D1_BASE @ result["lever_to_cad"], 7).tolist(),
        "lever_measured": {k: result[k] for k in ("shaft_from_holster_frame_mm", "bar_faces_off_planes_mm",
                                                  "lever_bounds_m", "lever_volume_cm3", "ward_angle_deg",
                                                  "shaft_radius_mm", "holster_volumes_cm3")},
        "inputs": result["inputs"],
        "holster_mesh": {"path": str(HOLSTER_MESH_PATH.relative_to(ROOT)), "sha256": _sha256(HOLSTER_MESH_PATH)},
        "lever_mesh": {"path": str(LEVER_V4_PATH.relative_to(ROOT)), "sha256": _sha256(LEVER_V4_PATH)},
        "replaced": previous,
    })
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


def holster_in_d1_base(record: dict | None = None) -> np.ndarray:
    """4x4: the holster's frame -- the key frame of v3 as the export holds it -- to the D1's `base_link`. The holster's
    boxes (`HOLSTER_BOXES`) and mesh are in it."""
    return np.asarray((record or load())["key_in_d1_base"], dtype=float)


def key_in_d1_base(record: dict | None = None, extra_m: float = LEVER_EXTRA_M) -> np.ndarray:
    """4x4: the key frame (module docstring) to the D1's `base_link`, with the key in the holster: v4's, as its STL
    has it (`register_v4`, the record's `lever_in_d1_base`); without one, the holster's frame moved `extra_m` out along
    the holster's axis, for a lever that much longer than v3 (`LEVER_EXTRA_M`)."""
    record = record or load()
    if "lever_in_d1_base" in record:
        return np.asarray(record["lever_in_d1_base"], dtype=float)
    frame = holster_in_d1_base(record).copy()
    frame[:3, 3] += extra_m * (frame[:3, :3] @ np.asarray(SLIDE_OUT_KEY))
    return frame


def key_boxes(extra_m: float | None = None) -> dict:
    """The holster's key's colliders in its frame: v4's (`KEY_BOXES`, its own length). With `extra_m`, v3's
    (`V3_KEY_BOXES`) for a lever that much longer: the bar's far end and the platform that much further from the shaft."""
    if extra_m is None:
        return dict(KEY_BOXES)
    out = dict(V3_KEY_BOXES)
    (lo, hi) = V3_KEY_BOXES["bar"]
    out["bar"] = (lo, (hi[0], hi[1] + extra_m, hi[2]))
    (lo, hi) = V3_KEY_BOXES["platform"]
    out["platform"] = ((lo[0], lo[1] + extra_m, lo[2]), (hi[0], hi[1] + extra_m, hi[2]))
    return out


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


def draw_soft(soft=IK_SOFT) -> tuple:
    """`soft` (one factor or six) with Joint1 and Joint5 at least `DRAW_SOFT_J1_J5`: the IK's limits out of the
    holster and back."""
    s = np.broadcast_to(np.asarray(soft, dtype=float), (6,)).copy()
    s[[0, 4]] = np.maximum(s[[0, 4]], DRAW_SOFT_J1_J5)
    return tuple(float(v) for v in s)


def exit_direction(axes: dict) -> np.ndarray:
    """The way out of the mouth after the draw along the axis: `EXIT_DEG` off it toward the front wall (`n`)."""
    a = math.radians(EXIT_DEG)
    return math.cos(a) * axes["out"] + math.sin(a) * axes["n"]


def clear_direction(axes: dict) -> np.ndarray:
    """The lift after the exit (`CLEAR_M`): `CLEAR_OUT_DEG` out along the axis, from across it `CLEAR_ROUND_DEG` round
    from `n` toward `t`."""
    a, b = math.radians(CLEAR_OUT_DEG), math.radians(CLEAR_ROUND_DEG)
    return math.cos(a) * (math.cos(b) * axes["n"] + math.sin(b) * axes["t"]) + math.sin(a) * axes["out"]


def draw_points(key: np.ndarray, roll: int = GRIP_ROLL, slide_m: float = SLIDE_M,
                grip_along_m: float = GRIP_ALONG_M, grip_onto_m: float = 0.0) -> dict:
    """The jaw centre for each step of the draw, in `key`'s frame, with the hand's rotation: `grip` on the bar
    `GRIP_ALONG_M` from the shaft, `pre` out along the axis, `out` drawn by `slide_m` along it (`SLIDE_M`), `exit`
    `EXIT_M` on from there out through the mouth (`exit_direction`)."""
    axes = key_axes(key)
    rot = grip_rotation(axes, roll)
    grip = (axes["origin"] + grip_along_m * axes["u"]
            + (KEY_ORIGIN_LINK6[2] - JAW_CENTRE_Z_M - grip_onto_m) * axes["n"])
    out = grip + slide_m * axes["out"]
    exit_ = out + EXIT_M * exit_direction(axes)
    return {"rot": rot, "pre": grip + PRE_OUT_M * axes["out"], "grip": grip, "out": out, "exit": exit_,
            "clear": exit_ + CLEAR_M * clear_direction(axes)}


def plates_in(frame: np.ndarray, plates: dict | None = None) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """The mouth's plates (`mouth_plates`, holster frame) as `boxes_in` gives boxes, in `frame`'s (the holster frame
    there)."""
    plates = mouth_plates() if plates is None else plates
    return [(frame[:3, :3] @ c + frame[:3, 3], frame[:3, :3] @ axes, half) for c, axes, half in plates.values()]


def boxes_in(key: np.ndarray, boxes: dict) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """The boxes (given in the key frame) as (centre, axes as columns, half sizes) in `key`'s frame."""
    out = []
    for lo, hi in boxes.values():
        lo, hi = np.asarray(lo), np.asarray(hi)
        out.append((key[:3, :3] @ ((lo + hi) / 2) + key[:3, 3], key[:3, :3], (hi - lo) / 2))
    return out


def box_points(box, step: float = 0.002) -> np.ndarray:
    """A grid of points through a box (centre, axes as columns, half sizes), its faces included, at most `step` apart."""
    centre, axes, half = box
    grids = [np.linspace(-h, h, max(2, int(math.ceil(2 * h / step)) + 1)) for h in np.asarray(half, dtype=float)]
    local = np.stack(np.meshgrid(*grids, indexing="ij"), -1).reshape(-1, 3)
    return local @ np.asarray(axes).T + centre


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
    holster_go2 = holster_in_d1_base().copy()
    holster_go2[:3, 3] += MOUNT_B
    pts = draw_points(key_go2, roll)
    rot = pts["rot"]
    offset = (0.0, 0.0, JAW_CENTRE_Z_M)
    rest = np.asarray(d1_ik.from_servo_deg(d1_hardware.REST_SERVO_DEG)[0], dtype=float)
    lows, highs = d1_ik.joint_limits(joints, 1.0)
    holster = boxes_in(holster_go2, HOLSTER_BOXES) + plates_in(holster_go2)
    moves = [("pre", pts["pre"], pts["pre"], JAW_OPEN_M), ("on", pts["pre"], pts["grip"], JAW_OPEN_M),
             ("out", pts["grip"], pts["out"], JAW_HOLD_M), ("exit", pts["out"], pts["exit"], JAW_HOLD_M),
             ("clear", pts["exit"], pts["clear"], JAW_HOLD_M)]
    q = None
    rows = []
    for name, start, end, jaw in moves:
        for f in np.linspace(0.0, 1.0, steps if name != "pre" else 1):
            point = start + f * (end - start)
            seeds = [q] if q is not None else [rest] + _seeds(point)
            result = None
            move_soft = draw_soft(soft) if name in ("out", "exit", "clear") else soft
            for seed in seeds:
                r = d1_ik.solve(joints, point, rot, q0=seed, body="Link6", offset=offset, max_iterations=400,
                                soft=move_soft)
                if r.converged and bool(arm_clear(joints, r.q)[0]):
                    result = r
                    break
            if result is None:
                rows.append({"move": name, "f": round(float(f), 2), "solved": False})
                return {"solved": False, "rows": rows, "points": pts}
            q = result.q
            slid = point - pts["grip"] if name in ("out", "exit", "clear") else np.zeros(3)
            arm = _arm_points(joints, q, jaw)
            hand = np.vstack([arm[k] for k in HAND_LINKS if k in arm])
            links_moving = np.vstack([v for k, v in arm.items() if k not in HAND_LINKS and k != "base_link"])
            moved = key_go2.copy()
            moved[:3, 3] += slid
            keys = boxes_in(moved, key_boxes())
            # Before the jaws close the hand must not touch the key: the open fingers either side of the bar.
            hand_to_key = (round(1000 * float(box_distance(hand, keys).min()), 1) if name in ("pre", "on")
                           else None)
            use = np.abs(gravity_torques(joints, links, q[None]))[0] / EFFORT_LIMIT_NM
            span = np.where(q >= 0, q / highs, q / lows)
            rows.append({"move": name, "f": round(float(f), 2), "solved": True, "q": np.round(q, 3).tolist(),
                         "key_to_mouth_mm": round(1000 * float(box_distance(
                             np.vstack([box_points(b) for b in keys]), plates_in(holster_go2)).min()), 1),
                         "range_use_max": round(float(span.max()), 3), "worst_joint": int(np.argmax(span)) + 1,
                         "gravity_use_max": round(float(use.max()), 3),
                         "trunk_proxy_margin_m": round(proxy_margin(joints, q), 3),
                         "hand_to_holster_mm": round(1000 * float(box_distance(hand, holster).min()), 1),
                         "open_hand_to_key_mm": hand_to_key,
                         "arm_to_holster_mm": round(1000 * float(box_distance(links_moving, holster).min()), 1),
                         "key_to_arm_base_mm": round(1000 * float(box_distance(arm["base_link"], keys).min()), 1),
                         "key_to_arm_links_mm": round(1000 * float(box_distance(links_moving, keys).min()), 1)})
    # The unfold (`unfold_stages`): from the folded rest to `pre`, each leg as the dog program sends it (`joint_move`),
    # against the holster, the key in it and the trunk proxy. (The check must model the path the arm is sent: an
    # earlier one modelled `d1_ik.traversal_configs` while the link drove straight joint lines, called the unfold 11.9 mm
    # clear, and on the straight line a finger went 3.3 mm into the key.)
    q_pre = np.asarray(rows[0]["q"])
    goals = unfold_stages(rest, q_pre, above_pre(joints, q_pre, key_go2, roll, soft))
    path = np.concatenate([joint_move(a, b) for a, b in zip([rest, *goals[:-1]], goals)])
    keys = boxes_in(key_go2, key_boxes())
    unfold = []
    for qq in path:
        arm = _arm_points(joints, qq, JAW_OPEN_M)
        moving = np.vstack([v for k, v in arm.items() if k != "base_link"])
        unfold.append(min(float(box_distance(moving, holster).min()), float(box_distance(moving, keys).min())))
    return {"solved": True, "rows": rows, "points": pts, "q_pre": q_pre, "q_above": goals[1], "rest": rest,
            "unfold_clearance_mm": round(1000 * min(unfold), 1),
            "unfold_trunk_proxy_margin_m": round(min(proxy_margin(joints, qq) for qq in path), 3)}


def joint_paced(joint_speed: float, roll: int = GRIP_ROLL, soft: float = IK_SOFT, samples: int = 41) -> dict:
    """The draw's three straight moves (`on`: pre to grip, `draw`: grip to out, `exit`: out to exit; the last two at
    `draw_soft`) timed so that the arm's fastest joint
    turns at `joint_speed` rad/s throughout, on the CPU model: {move: (seconds, u, s, q)}, where `s` (0 to 1 along the
    line) at the phase's progress `u` is `np.interp(u, u_table, s_table)`, and `q` (samples x 6, URDF radians) are the
    joints there.

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
    for move, a, b in (("on", pts["pre"], pts["grip"]), ("draw", pts["grip"], pts["out"]),
                       ("exit", pts["out"], pts["exit"])):
        move_soft = soft if move == "on" else draw_soft(soft)
        s = np.linspace(0.0, 1.0, samples)
        qs = []
        for f in s:
            point = a + f * (b - a)
            # As `check` seeds it: the folded rest first, then the scripted grip's seeds; then warm-started.
            for seed in ([q] if q is not None else [_rest_q()] + _seeds(point)):
                r = d1_ik.solve(joints, point, pts["rot"], q0=seed, body="Link6", offset=offset, max_iterations=400,
                                soft=move_soft)
                if r.converged and bool(arm_clear(joints, r.q)[0]):
                    break
            q = r.q
            qs.append(q)
        step = np.abs(np.diff(np.asarray(qs), axis=0)).max(axis=1)
        t = np.concatenate([[0.0], np.cumsum(step / joint_speed)])
        out[move] = (float(t[-1]), t / t[-1], s, np.asarray(qs))
    return out


def draw_plan(roll: int = GRIP_ROLL, soft: float = IK_SOFT, slide_m: float = SLIDE_M,
              grip_along_m: float = GRIP_ALONG_M, grip_onto_m: float = 0.0) -> dict:
    """The draw for a robot program, in the Go2's base frame (the IK's): its points (`pre`, `grip`, `out`, `exit`), the hand's
    rotation, and the joint goals of the unfold (`unfold_stages`: the turned fold, above `pre`, `pre`), the first of
    them for `rest`. Seeded as `check` is, so it is the branch `check` cleared. IK only (`check` adds the clearances).
    The key is drawn `slide_m` along the axis, then out through the mouth, the jaws closing `grip_along_m` along the
    bar from the shaft and `grip_onto_m` further onto it along the approach than `KEY_ORIGIN_LINK6` (`draw_points`)."""
    import d1_ik
    from demos.cup.pick_demo.grasp import _seeds, arm_clear
    from position_only.workspace import MOUNT_B, load_urdf

    joints, _ = load_urdf()
    key = key_in_d1_base().copy()
    key[:3, 3] += MOUNT_B
    pts = draw_points(key, roll, slide_m, grip_along_m, grip_onto_m)
    rest = _rest_q()
    q_pre = None
    for seed in [rest] + _seeds(pts["pre"]):
        r = d1_ik.solve(joints, pts["pre"], pts["rot"], q0=seed, body="Link6", offset=(0.0, 0.0, JAW_CENTRE_Z_M),
                        max_iterations=400, soft=soft)
        if r.converged and bool(arm_clear(joints, r.q)[0]):
            q_pre = np.asarray(r.q, dtype=float)
            break
    if q_pre is None:
        raise ValueError("the holster's waiting point does not solve")
    stages = unfold_stages(rest, q_pre, above_pre(joints, q_pre, key, roll, soft))
    return {"key": key, "axes": key_axes(key), "pre": pts["pre"], "grip": pts["grip"], "out": pts["out"],
            "exit": pts["exit"], "clear": pts["clear"], "rot": pts["rot"], "turned": stages[0], "above": stages[1], "q_pre": stages[2]}


def _rest_q() -> np.ndarray:
    import d1_hardware
    import d1_ik

    return np.asarray(d1_ik.from_servo_deg(d1_hardware.REST_SERVO_DEG)[0], dtype=float)


def unfold_stages(rest, q_pre, q_above) -> list[np.ndarray]:
    """The joint-space goals from the folded rest to the hand at `pre`: Joint1 turned first, the arm still folded; then
    the hand `ABOVE_PRE_M` above `pre` (`q_above`, `above_pre`); then `pre`. Each leg is a straight line in joint space,
    as the arm link's profile (`arm_link.Profile`) and the simulation's joint moves drive it. Straight from rest to
    `pre` the hand dips into the trunk proxy's 3 cm margin; from the turned fold straight to `pre` a finger passes
    through the holstered key's root (3.3 mm in, with the grip 1 cm higher). Through the point above, 14.0 and 16.7 mm
    clear, the trunk proxy 50 mm or more (the CPU model, 2026-10-23). With v5's key 15 mm further out, the folded hand
    turning round passed its shaft 2.7 mm off, so the elbow opens `TURN_ELBOW_RAD` as Joint1 turns: 11.4 mm, the rest
    pose's own clearance (and 17.3 to the mouth after it, the CPU model, 2026-10-24)."""
    turned = np.asarray(rest, dtype=float).copy()
    turned[0] = q_pre[0]
    turned[2] += TURN_ELBOW_RAD
    return [turned, np.asarray(q_above, dtype=float), np.asarray(q_pre, dtype=float)]


#: The unfold's waypoint: the hand this far straight above `pre`, as it will hold the bar (`unfold_stages`).
ABOVE_PRE_M = 0.08
#: The elbow (Joint3) opened this much from the fold as Joint1 turns (`unfold_stages`).
TURN_ELBOW_RAD = math.radians(-20.0)


def above_pre(joints, q_pre, key: np.ndarray | None = None, roll: int = GRIP_ROLL, soft: float = IK_SOFT):
    """The joints with the hand `ABOVE_PRE_M` above `pre` (Go2 base frame by default), warm-started from `q_pre`."""
    import d1_ik
    from position_only.workspace import MOUNT_B

    if key is None:
        key = key_in_d1_base().copy()
        key[:3, 3] += MOUNT_B
    pts = draw_points(key, roll)
    r = d1_ik.solve(joints, pts["pre"] + np.array([0.0, 0.0, ABOVE_PRE_M]), pts["rot"], q0=q_pre, body="Link6",
                    offset=(0.0, 0.0, JAW_CENTRE_Z_M), max_iterations=400, soft=soft)
    if not r.converged:
        raise ValueError(f"the point above pre does not solve ({1000 * r.position_error_m:.1f} mm)")
    return np.asarray(r.q, dtype=float)


#: The joint moves' pace on every joint while it moves, and how fast each gets there and stops (`joint_move`), per
#: second: 40 deg/s is 4 deg a 10 Hz command, past the ~2.5 the D1 runs through without stopping (F-045); the base
#: swivel at that pace was the one smooth move of the first live runs (Lukas, 2026-10-23).
JOINT_MOVE_RAD_S = math.radians(40.0)
JOINT_MOVE_ACCEL_RAD_S2 = math.radians(300.0)


def joint_move(a, b, speed: float = JOINT_MOVE_RAD_S, accel: float = JOINT_MOVE_ACCEL_RAD_S2,
               dt: float = 0.1) -> np.ndarray:
    """A joint-space move as the dog program sends it: each joint on its own, accelerating at `accel` to `speed`,
    cruising and braking onto its goal, so each takes its own time and all move at the same full pace while they move
    (not a straight line in joint space, on which the joints that move least step too little to run smoothly). Units
    are the caller's (radians, or servo degrees with `speed` and `accel` in degrees). Sampled every `dt` until the last
    joint arrives."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    dist = np.abs(b - a)
    ramp = np.minimum(speed / accel, np.sqrt(dist / accel))           # a triangle where a joint cannot reach cruise
    top = accel * ramp
    total = 2 * ramp + np.where(top > 0, (dist - top * ramp) / np.maximum(top, 1e-12), 0.0)
    t = np.append(np.arange(0.0, float(total.max()), dt), float(total.max()))[:, None]
    up = 0.5 * accel * np.minimum(t, ramp) ** 2
    cruise = top * np.clip(t - ramp, 0.0, np.maximum(total - 2 * ramp, 0.0))
    down_t = np.clip(t - (total - ramp), 0.0, ramp)
    down = top * down_t - 0.5 * accel * down_t ** 2
    covered = np.minimum(up + cruise + down, dist)
    return a + np.sign(b - a) * np.where(t >= total, dist, covered)


def joint_line(a, b, samples: int = 41) -> np.ndarray:
    """A straight line in joint space, as the arm link and the simulation's joint moves drive it."""
    return np.asarray(a, dtype=float) + np.linspace(0.0, 1.0, samples)[:, None] * (np.asarray(b) - np.asarray(a))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stl", nargs="*", help="d1_with_holster.stl d1_with_lever.stl d1_with_holster_and_lever.stl "
                                               "(10-23), or holster_lever.stl lever_only.stl (v4, 10-24), or "
                                               "holster_with_lever2.stl lever2.stl (v5, 10-24)")
    parser.add_argument("--write", action="store_true", help="write the holster mesh and the registration")
    parser.add_argument("--design", choices=tuple(DESIGNS), default="v5", help="two STLs: which design they are")
    parser.add_argument("--check", action="store_true", help="the draw on the CPU arm model")
    parser.add_argument("--roll", type=int, default=GRIP_ROLL)
    parser.add_argument("--soft", type=float, default=IK_SOFT)
    args = parser.parse_args(argv)
    if len(args.stl) == 2:
        result = register_v4(*args.stl)
        for k in ("shaft_from_holster_frame_mm", "bar_faces_off_planes_mm", "lever_bounds_m", "lever_volume_cm3",
                  "ward_angle_deg", "shaft_radius_mm", "holster_volumes_cm3"):
            print(f"{k}: {result[k]}")
        if args.write:
            record = write_v4(result, args.design)
            print(f"wrote {REGISTRATION_PATH.relative_to(ROOT)}, {record['holster_mesh']['path']} and "
                  f"{record['lever_mesh']['path']}")
    elif args.stl:
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

"""The RealSense's physical body, so the assumed wrist mount can be checked against the real bracket.

The wrist camera has always been an invisible `CameraCfg` prim at `WristMount.pose`: the pick's geometry
used it, but nothing in the viewer showed where the camera *is*. With a bracket now built, the question
"does the simulated camera sit where the real one sits" needs a body to look at.

**This is Intel's own CAD**, not a box drawn from datasheet dimensions. `pick_demo/assets/realsense/
d435_housing.ply` is the D435 case mesh from the ROS package `realsense2_description`, licence and
provenance in `third_party/realsense2_description/NOTICE.md`. The D435i shares this case:
`_d435i.urdf.xacro` builds the D435i by including `_d435.urdf.xacro` unchanged and adding IMU frames.

Everything here is in the **colour optical frame** -- x right, y down, z forward, the frame
`CameraModel` describes and `WristMount.pose` places -- because that is the frame the pick's geometry
already uses.

Where the mesh sits in that frame (`MESH_TO_OPTICAL`) is not a guess. `_d435.urdf.xacro` states it:
the mesh origin is on the front plate at the case's long-axis centre (the tripod screw axis), the
depth origin is `d435_cam_depth_py` = 17.5 mm along the case from there, the colour sensor is
`d435_cam_depth_to_color_offset` = 15 mm further, and the optical centres lie
`d435_zero_depth_to_glass` = 4.2 mm behind the glass, itself `d435_glass_to_front` = 0.1 mm behind the
plate. Applying that transform lands the mesh's own lens barrels where the extrinsics say they are --
the three concentric colour-lens parts at x = 0.00 mm, the left imager at +15.25 mm and the right at
+65.25 mm -- which is the registration checking itself rather than being asserted.
`tests/test_pick_demo.py` holds those numbers.

Where each number comes from, in the project's usual labels:

* **measured** -- read off the connected camera through librealsense (`pick_demo.realsense`), from
  D435I 238222076237: the colour sensor sits 14.857 mm from the depth origin, and the stereo baseline
  is 50.05 mm. Both agree with the xacro's nominal 15 mm and 50 mm to a quarter of a millimetre.
* **CAD** -- the mesh itself, and the xacro offsets that place it.
* **datasheet** -- Intel D400-series datasheet, Rev 014, for the nominal 90 x 25 x 25 mm case. The CAD
  measures 89.91 x 25.00 x 25.06 mm, which is the number this module now uses.

**The sign that was wrong.** Until 2026-09-17 this module put the left imager at *minus*
`COLOUR_FROM_LEFT_IMAGER_M` and centred the case on the stereo pair, which mirrored the body along its
long axis and put its centre 22 mm from where the CAD puts it. librealsense's
`depth.get_extrinsics_to(colour)` returns `p_colour = R p_depth + t`, so `t_x = +14.857 mm` is the
depth origin's position *in the colour frame*: the left imager is to the **right** of colour, and the
case runs from -12.5 mm to +77.4 mm about it. The xacro agrees. `clearance_report` -- the number a
bracket is designed around -- was wrong by that much.

The body is **visual only**: no collider, no mass, no rigid-body properties. It is a child prim of
Link6, so it follows the wrist, and it cannot change the physics the pick was measured against.

**It can occlude the wrist camera, and with the renderer as it stands it does.** A pinhole camera at
the sensor plane looks out through 4.3 mm of its own case, so the colour lens element sits on its
axis; `camera_asset` removes those triangles when it builds the USD. That is enough only if the
rendered camera is where the mount puts it, and F-052 says it is not: the renderer's eye sits about
9 mm below and 6 mm behind the optical origin, which is *inside* the case. `view_obstruction` takes
that offset as an argument for exactly this reason -- asked about the model frame it reports clear,
asked about the rendered frame it reports the case. Before F-052 the guard only ever asked the model
frame, which is why it certified a clear view of a lens marker that was plainly in shot.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

ASSET_DIR = Path(__file__).resolve().parent / "assets" / "realsense"
MESH_PATH = ASSET_DIR / "d435_housing.ply"

# Measured on D435I 238222076237 (pick_demo.realsense calibrate, 2026-09-17): the depth origin -- the
# left infrared imager -- sits this far along the colour frame's +x. Positive: to the right of colour.
COLOUR_FROM_LEFT_IMAGER_M = 0.014857
# Measured on the same camera, from the two infrared streams' extrinsics: 50.05 mm, against the
# datasheet's nominal 50 mm.
STEREO_BASELINE_M = 0.05004734918475151

# From realsense2_description/urdf/_d435.urdf.xacro, which cites the datasheet Rev 007 Fig. 4-4 p.65.
GLASS_AHEAD_OF_OPTICAL_M = 0.0042        # optical centres sit this far behind the front cover glass
GLASS_BEHIND_PLATE_M = 0.0001            # glass this far behind the front aluminium plate
MESH_DEPTH_FROM_SCREW_M = 0.0175         # depth origin, along the case, from the tripod screw axis
NOMINAL_COLOUR_FROM_DEPTH_M = 0.015      # the xacro's nominal for COLOUR_FROM_LEFT_IMAGER_M
MOUNT_FROM_CENTRE_M = 0.0149             # tripod screw, into the case from the front plate
# The mesh origin's offset from the depth origin along the optical axis, as the xacro composes it.
MESH_ORIGIN_BEHIND_OPTICAL_M = MOUNT_FROM_CENTRE_M - GLASS_BEHIND_PLATE_M - GLASS_AHEAD_OF_OPTICAL_M
# Where the mesh's long-axis origin (the tripod screw axis) sits in the colour optical frame. The
# xacro's nominal colour offset is used, not the measured one, so that the CAD registers against
# itself: the mesh's own colour lens barrel then lands on the optical axis, which is the check in
# `tests/test_pick_demo.py`. The bench camera's measured 14.857 mm differs from the nominal 15 mm by
# 0.14 mm -- device tolerance, not a registration choice.
SCREW_TO_COLOUR_M = MESH_DEPTH_FROM_SCREW_M + NOMINAL_COLOUR_FROM_DEPTH_M

# Datasheet nominal, kept for the docstring's sake; the CAD's own extent is what the code uses.
HOUSING_SIZE_NOMINAL_M = (0.090, 0.025, 0.025)


def mesh_to_optical() -> np.ndarray:
    """4x4 taking a point of `d435_housing.ply` into the colour optical frame.

    Derived from `_d435.urdf.xacro`. The mesh's own axes are x along the case, y up, z out of the
    front plate. The xacro mounts it in the camera link with rpy (pi/2, 0, pi/2) at
    (glass + plate, -depth_py, 0), and the colour optical frame is the link rotated by
    rpy (-pi/2, 0, -pi/2) at (0, +colour_offset, 0). Composing the two gives, for a mesh point p:

        optical x =  (screw_to_colour) - p.x        with screw_to_colour = depth_py + colour offset
        optical y = -p.y
        optical z =  p.z + glass + plate

    The rotation is a half turn about the optical z: the mesh runs along its +x away from the colour
    lens, the optical frame's +x runs back towards it, and the mesh's +y is up where the optical +y is
    down.
    """
    out = np.array([
        [-1.0, 0.0, 0.0, SCREW_TO_COLOUR_M],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, GLASS_AHEAD_OF_OPTICAL_M + GLASS_BEHIND_PLATE_M],
        [0.0, 0.0, 0.0, 1.0],
    ])
    return out


MESH_TO_OPTICAL = mesh_to_optical()


@lru_cache(maxsize=1)
def mesh_optical() -> tuple:
    """The case mesh in the colour optical frame, as (points (N,3), triangles (T,3)).

    Cached: the file is 7 MB and 231186 triangles, and every caller wants the same transform applied.
    Needs `trimesh`, which the Isaac environment has.
    """
    import trimesh

    mesh = trimesh.load(str(MESH_PATH))
    points = np.asarray(mesh.vertices, dtype=float)
    points = (MESH_TO_OPTICAL @ np.c_[points, np.ones(len(points))].T).T[:, :3]
    return points, np.asarray(mesh.faces, dtype=np.int64)


def housing_bounds_m() -> tuple:
    """(min, max) corner of the case in the colour optical frame, from the CAD."""
    points, _ = mesh_optical()
    return tuple(points.min(axis=0)), tuple(points.max(axis=0))


def housing_centre_m() -> tuple:
    lo, hi = housing_bounds_m()
    return tuple((a + b) / 2.0 for a, b in zip(lo, hi))


def housing_size_m() -> tuple:
    lo, hi = housing_bounds_m()
    return tuple(b - a for a, b in zip(lo, hi))


def imager_positions_m() -> dict:
    """Each optical centre in the colour optical frame, on the front-glass plane.

    The colour sensor is the origin by construction. The left imager is the depth origin, the measured
    offset to its **right** (see the module docstring on the sign). The right imager is a measured
    baseline further right again.
    """
    left_x = COLOUR_FROM_LEFT_IMAGER_M
    return {
        "left_ir": (left_x, 0.0, GLASS_AHEAD_OF_OPTICAL_M),
        "colour": (0.0, 0.0, GLASS_AHEAD_OF_OPTICAL_M),
        "right_ir": (left_x + STEREO_BASELINE_M, 0.0, GLASS_AHEAD_OF_OPTICAL_M),
    }


def tripod_thread_m() -> tuple:
    """The 1/4-20 thread's centre on the case's bottom face, in the colour optical frame.

    The xacro builds the whole camera off this feature -- it is the origin of `bottom_screw_frame` --
    so unlike the old estimate this is CAD, not a reading of the layout. It is the feature a bracket
    is actually bolted to.
    """
    lo, hi = housing_bounds_m()
    return (float(SCREW_TO_COLOUR_M), float(hi[1]), float(-MESH_ORIGIN_BEHIND_OPTICAL_M))


def part_pose_link6(point_m, mount) -> np.ndarray:
    """A point, given in the optical frame, expressed in the Link6 frame.

    `mount` is a `WristMount` or `MeasuredMount`: anything with a 4x4 `pose` placing the optical frame
    in Link6. This is how the spawner turns the geometry above into a prim offset.
    """
    point = np.asarray(point_m, dtype=float).reshape(3)
    return (np.asarray(mount.pose, dtype=float) @ np.append(point, 1.0))[:3]


def _half_extent(model, z: float) -> tuple:
    """Half-width and half-height of the colour frustum at depth `z`, from the real principal point.

    The image is not centred on the axis when cx and cy are not the image centre, so the frustum is
    asymmetric: what matters for occlusion is how far it reaches on the wider side. Note that the
    *renderer* ignores the principal point and centres it (F-052's companion measurement), so this
    over-reports against the render, which is the right way round for a guard.
    """
    across = max(model.cx, model.width - model.cx) / model.fx
    down = max(model.cy, model.height - model.cy) / model.fy
    return float(across * z), float(down * z)


# F-052: the rendered camera's eye, in the optical frame the mount defines, at the pick's observation
# pose. Measured from `camera_model_vs_sim` in the 2026-09-17 07:09 run: 10.86 mm total, of which
# 8.85 mm is down and 6.23 mm back -- inside the case. Ask the guard about this eye, not the origin,
# to learn what the renderer will actually see.
RENDERED_EYE_OFFSET_M = (0.0009, 0.00885, -0.00623)


def view_obstruction(model, eye_offset_m=(0.0, 0.0, 0.0), points=None) -> list:
    """Every part of the case that reaches into the colour frustum seen from `eye_offset_m`.

    Empty means the camera looks out of a clear aperture. The default eye is the optical origin, which
    is where the *model* puts the camera; pass `RENDERED_EYE_OFFSET_M` to ask where the *renderer*
    puts it. Those two answers differ, and the difference is the bug that put a lens marker in shot
    while this function reported clear.

    Conservative: a triangle counts as obstructing if any of it lies at positive depth and its x and y
    spans both overlap the frustum measured at the far end of its own depth range, which is where the
    frustum is widest. That over-reports rather than under-reports.

    Returns one entry per connected run of offending triangles is more than this needs, so it returns
    a single summary entry when anything is found, carrying the count and the extent of the intrusion.
    """
    if points is None:
        points, faces = mesh_optical()
        tri = points[faces]
    else:
        tri = np.asarray(points, dtype=float)
        if tri.ndim == 2:                      # a point cloud: treat each point as a degenerate triangle
            tri = tri[:, None, :].repeat(3, axis=1)
    eye = np.asarray(eye_offset_m, dtype=float).reshape(3)
    local = tri - eye
    zmax = local[:, :, 2].max(axis=1)
    ahead = zmax > 0.0
    if not ahead.any():
        return []
    across = max(model.cx, model.width - model.cx) / model.fx
    down = max(model.cy, model.height - model.cy) / model.fy
    hx, hy = across * zmax, down * zmax
    hit = (ahead
           & (local[:, :, 0].min(axis=1) <= hx) & (local[:, :, 0].max(axis=1) >= -hx)
           & (local[:, :, 1].min(axis=1) <= hy) & (local[:, :, 1].max(axis=1) >= -hy))
    if not hit.any():
        return []
    blocking = local[hit]
    return [{
        "part": "housing",
        "triangles": int(hit.sum()),
        "of": int(len(tri)),
        "eye_offset_mm": [round(float(v) * 1000.0, 2) for v in eye],
        "depth_range_m": [round(float(blocking[:, :, 2].min()), 5), round(float(blocking[:, :, 2].max()), 5)],
        "x_range_m": [round(float(blocking[:, :, 0].min()), 5), round(float(blocking[:, :, 0].max()), 5)],
        "y_range_m": [round(float(blocking[:, :, 1].min()), 5), round(float(blocking[:, :, 1].max()), 5)],
    }]


def clearance_report(mount, palm_z_m: float, palm_x_range_m) -> dict:
    """Does the case fit where the mount puts it, against Link6's own shell?

    The bracket has to hold the camera clear of the wrist. `grasp.py` carries the Link6 shell's CAD
    extent; this checks the case's eight bounding corners against it and reports the worst overlap, so
    a mount that buries the camera in the wrist is visible as a number rather than only in the viewer.

    **The shell check covers x and z only**, because those are the extents `grasp.py` has. At the
    default mount the optical x axis maps to Link6 -y, so the case's 90 mm long axis lies along Link6
    y and is *not* checked against anything: `housing_y_link6_m` is reported so it is at least visible.
    F-055's 22 mm correction moved the case along exactly that axis, which is why the x and z numbers
    here barely changed when it was applied.
    """
    lo, hi = (np.array(v, dtype=float) for v in housing_bounds_m())
    corners_optical = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    corners = np.array([part_pose_link6(c, mount) for c in corners_optical])
    x_lo, x_hi = float(corners[:, 0].min()), float(corners[:, 0].max())
    z_lo, z_hi = float(corners[:, 2].min()), float(corners[:, 2].max())
    # The shell occupies palm_x_range across the approach axis, out to palm_z along it.
    shell_x_lo, shell_x_hi = float(palm_x_range_m[0]), float(palm_x_range_m[1])
    overlap_x = min(x_hi, shell_x_hi) - max(x_lo, shell_x_lo)
    overlap_z = min(z_hi, palm_z_m) - max(z_lo, 0.0)
    intersects = overlap_x > 0.0 and overlap_z > 0.0
    return {
        "housing_x_link6_m": [round(x_lo, 4), round(x_hi, 4)],
        "housing_y_link6_m": [round(float(corners[:, 1].min()), 4), round(float(corners[:, 1].max()), 4)],
        "housing_z_link6_m": [round(z_lo, 4), round(z_hi, 4)],
        "y_checked": False,      # grasp.py has no shell extent along Link6 y; see the docstring

        "shell_x_link6_m": [round(shell_x_lo, 4), round(shell_x_hi, 4)],
        "shell_z_link6_m": [0.0, round(float(palm_z_m), 4)],
        "intersects_shell": bool(intersects),
        "overlap_m": [round(float(max(overlap_x, 0.0)), 4), round(float(max(overlap_z, 0.0)), 4)],
    }


def describe(mount) -> list:
    """Human-readable lines for the run log: where each feature lands in Link6."""
    lo, hi = housing_bounds_m()
    size = housing_size_m()
    lines = [f"RealSense D435i body at the wrist mount ({type(mount).__name__})",
             f"  case  {1000 * size[0]:.2f} x {1000 * size[1]:.2f} x {1000 * size[2]:.2f} mm"
             f"   [CAD: realsense2_description d435.dae]"]
    for name, point in (("case_min", lo), ("case_max", hi)):
        pos = part_pose_link6(point, mount)
        lines.append(f"  {name:14s} Link6 ({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}) m")
    for name, point in imager_positions_m().items():
        pos = part_pose_link6(point, mount)
        source = ("the optical frame itself" if name == "colour"
                  else "measured on D435I 238222076237")
        lines.append(f"  {name:14s} Link6 ({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}) m   [{source}]")
    thread = part_pose_link6(tripod_thread_m(), mount)
    lines.append(f"  {'tripod_1/4-20':14s} Link6 ({thread[0]:+.4f}, {thread[1]:+.4f}, {thread[2]:+.4f}) m"
                 "   [CAD: the xacro's bottom_screw_frame]")
    return lines


__all__ = ["mesh_to_optical", "MESH_TO_OPTICAL", "MESH_PATH", "mesh_optical", "housing_bounds_m",
           "housing_centre_m", "housing_size_m", "imager_positions_m", "tripod_thread_m",
           "part_pose_link6", "view_obstruction", "clearance_report", "describe",
           "RENDERED_EYE_OFFSET_M", "COLOUR_FROM_LEFT_IMAGER_M", "STEREO_BASELINE_M",
           "GLASS_AHEAD_OF_OPTICAL_M", "GLASS_BEHIND_PLATE_M", "HOUSING_SIZE_NOMINAL_M"]

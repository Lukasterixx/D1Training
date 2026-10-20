"""The Livox MID-360 on the Go2's trunk, simulated as a ray caster, and the door's face read off its points.

The combiner door demo uses the lidar the way P2Dingo's docking does (`dock_detector_node.cpp`, `dock_wall.hpp`): the
AprilTag gives a position, the lidar points around it give the heading. A 100 mm tag's heading shows only as the
difference between its left and right edges in the image, so it is its weakest number; a flat face round it is many
points on a line, whose direction is good to a fraction of a degree. Here that face is the door's own, the tag being
on the door:
- **at the look**, with the door shut, the face's line gives the box's heading (`perception.lidar_estimate`);
- **while the door opens**, the same line laid where the tag should be at the last reading gives the door's angle
  (`door_angle`), a check on the door that neither the script nor the hand provides. It steers nothing.

**Where it is.** The housing drawn on the trunk (`payload.py`, from Lukas's Go2 CAD): 65 mm square, its axis at
x 0.1758, y -0.0002, its bottom face at z 0.0961 in `base_link`. Livox puts the point-cloud origin on the axis 47.0 mm
above the bottom face (Mid-360 user manual, "Dimensions"), so the origin is (0.1758, -0.0002, 0.1431), the sensor's axes
those of `base_link`. P2Dingo, whose MID-360 sits in a different cage, uses (0.176, 0, 0.140) by the same rule.

**What it sees.** The datasheet's field, 360 x (-7 to +52) deg, cast as a 1 x 1 deg grid: 21,600 rays a frame at 10 Hz,
about the 20,000 points a 0.1 s frame holds at 200,000 points/s. The real pattern is non-repetitive and denser near
the centre of the field; this grid is a stand-in with the same density, not the pattern, and a fixed grid means frames
added together gain only in noise, not in coverage. Returns nearer than the 0.1 m blind zone are dropped, so the lidar's
own cage shadows without returning, as P2Dingo models it. Each return gets Gaussian range noise of `RANGE_NOISE_M`.
The rays are cast instantly from one pose, so a real 0.1 s sweep's motion distortion on a walking robot is not
modelled. They hit the box, the door, the handle, the arm's links and the lidar's cage, nothing else (no ground).

**The fit** (`face_line`) is `dock_wall::fitOnce` ported to numpy, with P2Dingo's structure and its numbers where the
door allows them (`FaceParams`): a window round the tag, along its face either side and from below its centre to above
it, from just in front of the face to just behind it, the handle left out; in it a RANSAC line that runs along the face
(within `max_disagree_deg` of the heading the window was laid at), refined twice by total least squares; then fitted
again in a window laid along the line it found, the line with more support kept. The window is the door's, not a
wall's: 0.28 m either side (the door is 0.63 wide), from 0.08 m in front of the face to 0.10 m behind it (behind the
door is the box's inside, uncovered as it opens), a 0.35 m span asked for against P2Dingo's 0.6 m, and an inlier band
of 6 cm against its 2.5, for the raw returns' noise (`FaceParams.inlier_m`).

The tag's centre sits about at the lidar's -7 deg lower edge when the robot stands at the door, so mostly the face
above it is seen.

Numpy at import; `lidar_cfg` imports Isaac when called.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from demos.combiner.geometry import GEOMETRY

#: The point-cloud origin in `base_link`: the housing's axis and Livox's 47 mm above its bottom (module docstring).
ORIGIN_BASE_M = (0.1758, -0.0002, 0.0961 + 0.047)
#: Livox Mid-360 datasheet: vertical field -7 to +52 deg, 360 deg around; 0.1 m blind zone; 40 m at 10% reflectivity.
VERTICAL_FOV_DEG = (-7.0, 52.0)
HORIZONTAL_FOV_DEG = (-180.0, 180.0)
RESOLUTION_DEG = 1.0
BLIND_ZONE_M = 0.1
MAX_RANGE_M = 40.0
#: Range noise, 1 sigma. The datasheet gives <= 3 cm at 0.2 m and <= 2 cm at 10 m; the door is ~0.4-0.6 m away.
RANGE_NOISE_M = 0.03
FRAME_PERIOD_S = 0.1


@dataclass(frozen=True)
class FaceParams:
    """`dock_wall::Params`, for a door rather than the wall behind a dock (module docstring)."""
    z_min_above_tag: float = -0.10
    z_max_above_tag: float = 0.40
    half_width_m: float = 0.28
    # The handle, left out as P2Dingo leaves the dock's stand out: its spindle 0.2125 m over the tag's centre, the
    # 40 mm tall lever reaching 0.12 m along the face from it, the hand gripping it.
    handle_half_width_m: float = 0.15
    handle_z_m: tuple = (0.15, 0.28)
    depth_min_m: float = -0.08
    depth_max_m: float = 0.10
    # P2Dingo's 2.5 cm suits its points, GLIM's map, 9 mm rms on its wall. These are raw returns with
    # `RANGE_NOISE_M` (3 cm) of noise: a band narrower than the noise keeps a skewed slice of it and tips the line
    # towards wherever the window was laid (4.3 deg from a window 10 deg off, in the tests). About 2 sigma.
    inlier_m: float = 0.06
    min_points: int = 40
    min_span_m: float = 0.35
    max_disagree_deg: float = 25.0
    iterations: int = 250
    max_points: int = 4000


DOOR = FaceParams()


@dataclass
class FaceFit:
    ok: bool
    heading: float = math.nan     # of a robot facing the face, rad (`dock_wall::Fit::heading`)
    depth: float = math.nan       # the face this far behind the window's centre, m
    points: int = 0
    span: float = 0.0
    rms: float = math.nan
    why: str = ""


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _fit_once(xy, z, centre, psi: float, psi_seed: float, p: FaceParams, rng) -> FaceFit:
    """`dock_wall::fitOnce`: one window laid at heading `psi` round `centre`, a line kept within `max_disagree_deg`
    of `psi_seed`."""
    hx, hy = math.cos(psi), math.sin(psi)        # into the face
    fx, fy = math.sin(psi), -math.cos(psi)       # along it, the viewer's right
    q = xy - centre[:2]
    dz = z - centre[2]
    u, d = q[:, 0] * fx + q[:, 1] * fy, q[:, 0] * hx + q[:, 1] * hy
    keep = ((dz >= p.z_min_above_tag) & (dz <= p.z_max_above_tag) & (np.abs(u) <= p.half_width_m)
            & (d >= p.depth_min_m) & (d <= p.depth_max_m)
            & ~((np.abs(u) < p.handle_half_width_m) & (dz >= p.handle_z_m[0]) & (dz <= p.handle_z_m[1])))
    pts = xy[keep]
    if p.max_points and len(pts) > p.max_points:
        pts = pts[(np.arange(p.max_points) * (len(pts) / p.max_points)).astype(int)]
    n = len(pts)
    if n < p.min_points:
        return FaceFit(False, why="too few points round the tag")
    seed = np.array([math.cos(psi_seed), math.sin(psi_seed)])
    max_off = math.sin(math.radians(p.max_disagree_deg))
    best, best_line = 0, None
    for i, j in rng.integers(0, n, size=(p.iterations, 2)):
        v = pts[j] - pts[i]
        length = float(np.hypot(*v))
        if length < 0.1:
            continue
        if abs(float(v @ seed)) / length > max_off:     # a line running along the face, not across it
            continue
        normal = np.array([-v[1], v[0]]) / length
        count = int((np.abs((pts - pts[i]) @ normal) <= p.inlier_m).sum())
        if count > best:
            best, best_line = count, (normal, pts[i])
    if best < p.min_points:
        return FaceFit(False, why="no line along the face")
    normal, origin = best_line
    for _ in range(2):          # total least squares on the inliers, twice
        inliers = pts[np.abs((pts - origin) @ normal) <= p.inlier_m]
        if len(inliers) < p.min_points:
            return FaceFit(False, why="the line lost its points")
        origin = inliers.mean(axis=0)
        _, _, vt = np.linalg.svd(inliers - origin, full_matrices=False)
        normal = vt[1]
    along = (inliers - origin) @ vt[0]
    span = float(along.max() - along.min())
    if span < p.min_span_m:
        return FaceFit(False, points=len(inliers), span=span, why="too short a line")
    if normal @ seed < 0:          # the heading points into the face, as the window's did
        normal = -normal
    rms = float(np.sqrt(np.mean(((inliers - origin) @ normal) ** 2)))
    depth = float((origin - centre[:2]) @ normal)
    return FaceFit(True, math.atan2(normal[1], normal[0]), depth, len(inliers), span, rms)


def face_line(points, centre, psi: float, params: FaceParams = DOOR, seed: int = 12345) -> FaceFit:
    """The face round `centre` (x, y, z) in `points` (n, 3), its window first laid at heading `psi` (a robot facing it),
    then laid again along the line found; the fit with more support. Any frame, so long as z is up."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    centre = np.asarray(centre, dtype=float)
    rng = np.random.default_rng(seed)
    first = _fit_once(points[:, :2], points[:, 2], centre, psi, psi, params, rng)
    if not first.ok:
        return first
    second = _fit_once(points[:, :2], points[:, 2], centre, first.heading, psi, params, rng)
    return second if second.ok and second.points >= first.points else first


# --- the door's angle while it opens -----------------------------------------------------------------------------

@dataclass
class DoorReading:
    angle_deg: float          # nan without a reading
    points: int               # on the face's line
    rms_m: float              # their distance from it, RMS
    offset_m: float           # the face's distance from where the box estimate puts the tag: the estimate's own check

    @property
    def ok(self) -> bool:
        return math.isfinite(self.angle_deg)


NO_READING = DoorReading(math.nan, 0, math.nan, math.nan)
#: Where else to lay the window when the last reading's window finds no face: across the door's travel.
RESCAN_DEG = tuple(float(a) for a in range(0, 115, 15))


def tag_at(angle_deg: float, geometry=GEOMETRY) -> np.ndarray:
    """The tag's centre in the enclosure frame with the door `angle_deg` open."""
    hx, hy = geometry.hinge[0], geometry.hinge[1]
    tx, ty, tz = geometry.tag_centre
    a = math.radians(geometry.hinge_axis_z * angle_deg)
    x, y = tx - hx, ty - hy
    return np.array([hx + math.cos(a) * x - math.sin(a) * y, hy + math.sin(a) * x + math.cos(a) * y, tz])


def facing(angle_deg: float, geometry=GEOMETRY) -> float:
    """The heading of a robot facing the door's outer face with the door `angle_deg` open, in the enclosure frame:
    the shut door's outward normal is +x, so facing it is pi, and the door turns it by `hinge_axis_z * angle`."""
    return _wrap(math.pi + math.radians(geometry.hinge_axis_z * angle_deg))


def door_angle(points_enclosure, prior_deg: float, geometry=GEOMETRY, params: FaceParams = DOOR) -> DoorReading:
    """The door's angle from lidar returns in the (estimated) enclosure frame: the face's line fitted round where the
    tag is at `prior_deg` (the last reading), and if no face is there, round where it is at each of `RESCAN_DEG`, the
    best-supported kept; then fitted again round where the tag is at the angle found."""
    def read(prior):
        fit = face_line(points_enclosure, tag_at(prior, geometry), facing(prior, geometry), params)
        if not fit.ok:
            return None
        angle = geometry.hinge_axis_z * math.degrees(_wrap(fit.heading - math.pi))
        return DoorReading(angle, fit.points, fit.rms, fit.depth)

    found = read(prior_deg)
    if found is None:
        readings = [r for r in (read(a) for a in RESCAN_DEG if abs(a - prior_deg) > 1e-6) if r is not None]
        if not readings:
            return NO_READING
        found = max(readings, key=lambda r: r.points)
    # Once more with the window round where the tag is at the angle found, as P2Dingo lays its window at the tag the
    # camera found rather than where it was expected: a window laid at a wrong angle clips the face unevenly.
    again = read(found.angle_deg)
    return again if again is not None else found


def lidar_cfg(targets, prim_path: str = "/World/envs/env_.*/Robot/base"):
    """The MID-360 as a `MultiMeshRayCaster` on the trunk, casting at `targets` (prim expressions), all tracked."""
    from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg, patterns

    return MultiMeshRayCasterCfg(
        prim_path=prim_path,
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=ORIGIN_BASE_M),
        ray_alignment="base",
        pattern_cfg=patterns.LidarPatternCfg(
            channels=int(round((VERTICAL_FOV_DEG[1] - VERTICAL_FOV_DEG[0]) / RESOLUTION_DEG)) + 1,
            vertical_fov_range=VERTICAL_FOV_DEG, horizontal_fov_range=HORIZONTAL_FOV_DEG,
            horizontal_res=RESOLUTION_DEG),
        mesh_prim_paths=[MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr=t, track_mesh_transforms=True)
                         for t in targets],
        max_distance=MAX_RANGE_M,
        update_period=FRAME_PERIOD_S,
        debug_vis=False,
    )

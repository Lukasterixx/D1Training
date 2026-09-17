"""Top-down cup grasps and camera viewpoints for the welded D1, planned with `d1_ik` (numpy only).

Why top-down. The pick starts with the Go2 lying on the floor, and the arm cannot bring a *level*
gripper down to a floor-standing cup from there: sampling 300k configurations inside the soft limits,
none puts the jaw centre within 8 cm of the floor with the approach axis within 15 deg of horizontal, at
any base height from 0.12 m (lying) to 0.27 m (standing). Pointing down, the jaws reach the upper part
of a ~10 cm cup 35-45 cm ahead of the base (Week 1 log, 2026-09-17). That fixes the grasp: fingers
straddle the cup body from above, jaw axis across the robot-to-cup line so a handle pointing away from
or towards the robot stays clear of both fingers.

Everything is in the Go2 base frame. The gripper geometry is the URDF's CAD (`workspace.tip_offsets`),
not a measurement of the real gripper.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

import d1_ik
from position_only.workspace import (BODY_BOX_B, EFFORT_LIMIT_NM, LINK_RADIUS_M, MOUNT_B, clear_of_body, forward,
                                     gravity_torques)

from .camera import CameraModel, WristMount, invert, transform
from .perception import CupEstimate, horizontal_basis

# Link6 frame, from the URDF CAD: the fingertips' end faces are 12.51 cm along the approach axis and
# the Link6 shell ends at 7.6 cm, so 4.95 cm of finger stands clear of the palm. The jaw centre is taken
# 2 cm back from the tips, mid-pad.
FINGERTIP_Z_M = 0.1251
PALM_Z_M = 0.076
PALM_X_RANGE_M = (-0.038, 0.019)   # the shell's extent across the approach axis, CAD
JAW_CENTRE_LINK6 = (0.0, 0.0, FINGERTIP_Z_M - 0.020)

# Per-finger travel of Joint7_1 (Joint7_2 mirrors it). 0 is closed; the URDF allows 3 cm. The CAD pads' inner
# faces are flat at |y| = 8.6 mm in Link6, so the gap runs from 17.2 mm closed to 77.2 mm open.
GRIPPER_OPEN_M = 0.03
GRIPPER_CLOSED_M = 0.0
CLOSED_GAP_M = 0.0172
OPEN_GAP_M = CLOSED_GAP_M + 2 * GRIPPER_OPEN_M

SEEDS = (
    np.zeros(6),
    np.array([0.0, 1.0, -0.5, 0.0, 1.2, 0.0]),
    np.array([0.0, 0.5, 0.5, 0.0, 1.0, 0.0]),
    np.array([0.0, 1.2, 0.3, 0.0, 0.8, 0.0]),
)


class PlanningError(RuntimeError):
    pass


@dataclass
class GraspParams:
    finger_overlap_m: float = 0.035      # fingertips this far below the rim at most
    rim_clearance_m: float = 0.015       # ...and never so far that the wrist shell comes within this of the rim
    min_overlap_m: float = 0.015         # a tilt that leaves less grip than this is not used
    # Start the descent this far back along the approach axis. It is also where the wrist camera looks again:
    # from 10 cm the gripper hides most of the cup and a D435 is inside its minimum depth (Week 1, 2026-09-17).
    pregrasp_clearance_m: float = 0.18
    lift_m: float = 0.12
    tilts_deg: tuple = (0.0, 10.0, 20.0, 30.0)
    # The jaw centre's allowed departure from a straight descent. The open jaws clear a 55 mm cup by 11 mm a
    # side; off the centre line some descents cannot get under ~4.3 mm however finely they are split.
    line_tolerance_m: float = 0.006
    lift_tolerance_m: float = 0.010      # looser once the cup is held: fewer stops on the way up
    # Longest single move on the way down and up. A descent the traversal model passes as one move still
    # overshoots at the bottom: in simulation one 18 cm move dropped the jaw 1.7 mm past its target and
    # the wrist shell pushed the cup 3.5 mm (Week 1 log, 2026-09-17).
    descend_step_m: float = 0.02
    lift_step_m: float = 0.04
    squeeze_m: float = 0.004             # close the jaws to this much less than the measured diameter


@dataclass
class GraspPlan:
    heading_b: np.ndarray
    tilt_deg: float
    rotation_b: np.ndarray
    jaw_pregrasp_b: np.ndarray
    jaw_grasp_b: np.ndarray
    jaw_lift_b: np.ndarray
    q_pregrasp: np.ndarray
    approach: list      # joint waypoints from the start pose to the pregrasp (a via pose, if one was needed)
    descend: list
    lift: list
    notes: list = field(default_factory=list)

    def as_dict(self) -> dict:
        r = lambda a: [round(float(v), 4) for v in np.asarray(a).ravel()]
        return {"heading_b": r(self.heading_b), "tilt_deg": self.tilt_deg,
                "jaw_pregrasp_b": r(self.jaw_pregrasp_b), "jaw_grasp_b": r(self.jaw_grasp_b),
                "jaw_lift_b": r(self.jaw_lift_b), "q_pregrasp": r(self.q_pregrasp),
                "approach_waypoints": len(self.approach), "descend_waypoints": len(self.descend),
                "lift_waypoints": len(self.lift), "notes": self.notes}


@dataclass
class ObservationPlan:
    q: np.ndarray
    camera_pose_b: np.ndarray
    look_at_b: np.ndarray
    distance_m: float
    elevation_deg: float
    roll_deg: float

    def as_dict(self) -> dict:
        return {"q": [round(float(v), 4) for v in self.q], "look_at_b": [round(float(v), 4) for v in self.look_at_b],
                "camera_pos_b": [round(float(v), 4) for v in self.camera_pose_b[:3, 3]],
                "distance_m": self.distance_m, "elevation_deg": self.elevation_deg, "roll_deg": self.roll_deg}


def holdable(joints, links, q) -> bool:
    return bool((np.abs(gravity_torques(joints, links, np.asarray(q).reshape(1, 6)))[0] <= EFFORT_LIMIT_NM).all())


def _chain_points(joints, qs) -> np.ndarray:
    """(N, P, 3): the points `workspace.clear_of_body` tests, five along each segment from Joint2 to the tip."""
    frames, _, origins = forward(joints, np.asarray(qs, dtype=float).reshape(-1, 6))
    rot6, pos6 = frames["Link6"]
    tip = pos6 + np.einsum("nij,j->ni", rot6, np.array([0.0, 0.0, 0.126]))
    chain = np.concatenate([origins[:, 1:], pos6[:, None], tip[:, None]], axis=1)
    t = np.linspace(0.0, 1.0, 5)[:, None, None, None]
    return (chain[None, :, :-1] * (1 - t) + chain[None, :, 1:] * t).transpose(1, 0, 2, 3).reshape(len(chain), -1, 3)


def arm_clear(joints, qs, base_height, up) -> np.ndarray:
    """`workspace.clear_of_body` with the ground where gravity puts it.

    The proxy takes the ground as the plane z = -base_height in the base frame, which is right only for a
    level base. Lying down, the simulated Go2 settles 7 deg nose-up (Week 1 log, 2026-09-17), and 44 cm
    ahead that level plane sits 5-6 cm above the real floor -- exactly where a cup's rim is, so every grasp
    read as underground. The body box is part of the base and stays as it is; the ground test uses `up`.
    """
    qs = np.asarray(qs, dtype=float).reshape(-1, 6)
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    box_clear = clear_of_body(joints, qs, math.inf)
    ground_clear = (_chain_points(joints, qs) @ up + base_height > LINK_RADIUS_M).all(axis=1)
    return box_clear & ground_clear


def path_clear(joints, q_start, q_goal, base_height, up):
    """`d1_ik.path_clearance` with `arm_clear`: (clear, first failing fraction or None)."""
    configs = d1_ik.traversal_configs(q_start, q_goal)
    ok = arm_clear(joints, configs, base_height, up)
    if bool(ok.all()):
        return True, None
    return False, float(np.flatnonzero(~ok)[0]) / max(1, len(ok) - 1)


def proxy_margin(joints, q, base_height, up) -> float:
    """How far (m) the arm is from failing `arm_clear`: the smallest gap between its link segments (same
    chain and radius as the proxy) and the body box or the ground. Negative inside."""
    points = _chain_points(joints, q)[0]
    low, high = np.array(BODY_BOX_B)[:, 0], np.array(BODY_BOX_B)[:, 1]
    outside = np.linalg.norm(np.maximum(np.maximum(low - points, points - high), 0.0), axis=1)
    inside = np.minimum(points - low, high - points).min(axis=1)
    box = np.where(outside > 0.0, outside, -np.maximum(inside, 0.0))
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    return float(min((box - LINK_RADIUS_M).min(), (points @ up + base_height - LINK_RADIUS_M).min()))


def jaw_positions(joints, qs) -> np.ndarray:
    frames, _, _ = forward(joints, np.asarray(qs, dtype=float).reshape(-1, 6))
    rot, pos = frames["Link6"]
    return pos + np.einsum("nij,j->ni", rot, np.asarray(JAW_CENTRE_LINK6))


def heading_to(point_b, up) -> np.ndarray:
    """Unit horizontal direction from the arm's mount to `point_b`."""
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    delta = np.asarray(point_b, dtype=float) - MOUNT_B
    delta -= up * (delta @ up)
    norm = np.linalg.norm(delta)
    if norm < 1e-6:
        raise PlanningError("target is directly above or below the arm's mount; no heading")
    return delta / norm


def top_down_rotation(heading, up, tilt_deg: float = 0.0) -> np.ndarray:
    """Link6 orientation (columns x, y, z) pointing down, jaw axis across `heading`.

    `tilt_deg` leans the approach axis away from the robot, which buys reach when straight down is not
    reachable: the fingertips then lead the wrist outwards.
    """
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    heading = np.asarray(heading, dtype=float)
    tilt = math.radians(tilt_deg)
    z = -math.cos(tilt) * up + math.sin(tilt) * heading
    y = np.cross(up, heading)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def solve_link6(joints, links, pos, rot, seeds, base_height, up, offset=JAW_CENTRE_LINK6):
    """First seed that converges, clears the trunk/ground proxy (`arm_clear`) and can be held against gravity."""
    for seed in seeds:
        result = d1_ik.solve(joints, pos, rot, q0=seed, body="Link6", offset=offset, max_iterations=300)
        if (result.converged and bool(arm_clear(joints, result.q, base_height, up)[0])
                and holdable(joints, links, result.q)):
            return result
    return None


def _seeds(point_b, extra=()):
    yaw = math.atan2(float(point_b[1]), float(point_b[0]))
    out = [np.asarray(s, dtype=float) for s in extra]
    for seed in SEEDS:
        for q1 in (0.0, yaw, -yaw):
            candidate = seed.copy()
            candidate[0] = q1
            out.append(candidate)
    return out


def line_deviation(points, start, end) -> np.ndarray:
    """Distance of each point from the segment start-end."""
    points = np.asarray(points, dtype=float)
    seg = np.asarray(end, dtype=float) - start
    length2 = float(seg @ seg)
    t = np.zeros(len(points)) if length2 < 1e-12 else np.clip((points - start) @ seg / length2, 0.0, 1.0)
    return np.linalg.norm(points - (start + t[:, None] * seg), axis=1)


def line_waypoints(joints, links, q_start, jaw_start, jaw_end, rot, base_height, up, tolerance_m: float = 0.004,
                   max_step_m: float | None = None, max_segments: int = 32) -> list:
    """Joint waypoints that carry the jaw centre along a straight line.

    The D1 moves each joint at its own speed ceiling (F-033), so one joint-space move between two IK
    solutions bends in Cartesian space -- through the cup, on a descent. The line is split until every
    traversal (as `d1_ik.traversal_configs` models it) stays within `tolerance_m` of it, and into steps no
    longer than `max_step_m` whatever the model says.
    """
    length = float(np.linalg.norm(np.asarray(jaw_end) - np.asarray(jaw_start)))
    segments = 1 if max_step_m is None else max(1, math.ceil(length / max_step_m - 1e-9))
    while segments <= max_segments:
        waypoints, previous, worst = [], np.asarray(q_start, dtype=float), 0.0
        for i in range(1, segments + 1):
            point = jaw_start + (jaw_end - jaw_start) * (i / segments)
            result = solve_link6(joints, links, point, rot, [previous, *_seeds(point)], base_height, up)
            if result is None:
                raise PlanningError(f"no IK solution {100 * i / segments:.0f}% along the line to "
                                    f"{np.round(jaw_end, 3).tolist()}")
            configs = d1_ik.traversal_configs(previous, result.q)
            worst = max(worst, float(line_deviation(jaw_positions(joints, configs), jaw_start, jaw_end).max()))
            clear, _ = path_clear(joints, previous, result.q, base_height, up)
            if not clear:
                raise PlanningError("a straight-line segment enters the trunk/ground proxy")
            if worst > tolerance_m:
                break
            waypoints.append(result.q)
            previous = result.q
        else:
            return waypoints
        segments *= 2
    raise PlanningError(f"straight line needs more than {max_segments} segments to stay within "
                        f"{1000 * tolerance_m:.0f} mm (worst {1000 * worst:.1f} mm)")


def plan_top_down_grasp(joints, links, cup: CupEstimate, up, q_start, base_height,
                        params: GraspParams | None = None) -> GraspPlan:
    params = params or GraspParams()
    if params.finger_overlap_m >= FINGERTIP_Z_M - PALM_Z_M:
        raise PlanningError("finger overlap would put the palm on the rim")
    if 2.0 * cup.radius_m > OPEN_GAP_M - 0.010:
        raise PlanningError(f"cup is {200 * cup.radius_m:.1f} cm wide; the open jaws are {100 * OPEN_GAP_M:.1f} cm")
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    heading = heading_to(cup.top_centre_b, up)
    reasons = []
    for tilt in params.tilts_deg:
        overlap = min(params.finger_overlap_m, shell_height_above_tips(tilt) - params.rim_clearance_m)
        if overlap < params.min_overlap_m:
            reasons.append(f"tilt {tilt:.0f}: the wrist shell would leave only {1000 * overlap:.0f} mm of grip")
            continue
        fingertip = cup.top_centre_b - up * overlap
        rot = top_down_rotation(heading, up, tilt)
        approach = rot[:, 2]
        jaw_grasp = fingertip - approach * (FINGERTIP_Z_M - JAW_CENTRE_LINK6[2])
        jaw_pre = jaw_grasp - approach * params.pregrasp_clearance_m
        jaw_lift = jaw_grasp + up * params.lift_m
        pre = solve_link6(joints, links, jaw_pre, rot, _seeds(jaw_pre, [q_start]), base_height, up)
        if pre is None:
            reasons.append(f"tilt {tilt:.0f}: pregrasp unreachable")
            continue
        approach, notes = approach_path(joints, links, q_start, pre.q, jaw_pre, rot, up, base_height)
        if approach is None:
            reasons.append(f"tilt {tilt:.0f}: every path to the pregrasp crosses the trunk/ground proxy")
            continue
        try:
            descend = line_waypoints(joints, links, pre.q, jaw_pre, jaw_grasp, rot, base_height, up,
                                     params.line_tolerance_m, params.descend_step_m)
            lift = line_waypoints(joints, links, descend[-1], jaw_grasp, jaw_lift, rot, base_height, up,
                                  params.lift_tolerance_m, params.lift_step_m)
        except PlanningError as exc:
            reasons.append(f"tilt {tilt:.0f}: {exc}")
            continue
        notes.append(f"fingertips {1000 * overlap:.0f} mm below the rim")
        return GraspPlan(heading, tilt, rot, jaw_pre, jaw_grasp, jaw_lift, pre.q, approach, descend, lift, notes)
    raise PlanningError("; ".join(reasons))


def shell_height_above_tips(tilt_deg: float) -> float:
    """How far above the fingertips (along up) the lowest point of the Link6 shell sits, for a top-down grasp.

    Tilting leans the approach axis away from the robot, which lowers the shell's robot-side edge (+x in
    Link6) by its offset times sin(tilt). Straight down that is the 4.9 cm of finger below the palm.
    """
    tilt = math.radians(tilt_deg)
    return (FINGERTIP_Z_M - PALM_Z_M) * math.cos(tilt) - max(PALM_X_RANGE_M[1] * math.sin(tilt),
                                                               PALM_X_RANGE_M[0] * math.sin(tilt))


def grip_travel_m(diameter_m: float, squeeze_m: float) -> float:
    """Per-finger travel that closes the jaws to `squeeze_m` less than the cup's diameter.

    Not simply "close". The URDF gives each finger its own drive, and commanded fully shut both saturate at
    their 15 N limit on contact. Two equal saturated forces hold the cup but do not centre it, so any
    imbalance walks it sideways: in simulation one finger pushed the cup 11 mm and drove the other back to its
    open stop (Week 1 log, 2026-09-17). A target just inside the cup keeps both drives below their limit,
    where each acts as a spring and the pair centres the cup. The real D1 moves both fingers from one servo,
    a coupling this model does not have.
    """
    return float(np.clip((diameter_m - squeeze_m - CLOSED_GAP_M) / 2.0, GRIPPER_CLOSED_M, GRIPPER_OPEN_M))


def approach_path(joints, links, q_start, q_goal, jaw_goal, rot, up, base_height, raises=(0.08, 0.16)):
    """Joint waypoints ending at `q_goal` whose every move clears the proxy, and notes; (None, notes) if none.

    Tries the direct move, then a via pose above the goal, then one above where the arm starts (backing
    out of a low start before swinging over), then both.
    """
    if path_clear(joints, q_start, q_goal, base_height, up)[0]:
        return [q_goal], []
    frames, _, _ = forward(joints, np.asarray(q_start, dtype=float).reshape(1, 6))
    start_rot, start_pos = frames["Link6"][0][0], frames["Link6"][1][0]
    start_jaw = start_pos + start_rot @ np.asarray(JAW_CENTRE_LINK6)
    candidates = []
    for rise in raises:
        above_goal = solve_link6(joints, links, jaw_goal + up * rise, rot, _seeds(jaw_goal, [q_goal, q_start]),
                                 base_height, up)
        above_start = solve_link6(joints, links, start_jaw + up * rise, start_rot, [q_start], base_height, up)
        if above_goal is not None:
            candidates.append(([above_goal.q, q_goal], f"via a pose {100 * rise:.0f} cm above the pregrasp"))
        if above_start is not None:
            candidates.append(([above_start.q, q_goal], f"backing {100 * rise:.0f} cm up out of the start pose"))
        if above_goal is not None and above_start is not None:
            candidates.append(([above_start.q, above_goal.q, q_goal], f"up {100 * rise:.0f} cm, across, and down"))
    for waypoints, note in candidates:
        previous, ok = q_start, True
        for q in waypoints:
            if not path_clear(joints, previous, q, base_height, up)[0]:
                ok = False
                break
            previous = q
        if ok:
            return waypoints, [f"direct move to the pregrasp crosses the proxy; {note}"]
    return None, []


def optical_rotation(view, up, roll_deg: float = 0.0) -> np.ndarray:
    """Optical frame (x right, y down, z = `view`) with the image upright, then rolled about the view."""
    z = np.asarray(view, dtype=float) / np.linalg.norm(view)
    down = -np.asarray(up, dtype=float)
    y = down - z * (down @ z)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    roll = math.radians(roll_deg)
    return np.column_stack([math.cos(roll) * x + math.sin(roll) * y, -math.sin(roll) * x + math.cos(roll) * y, z])


def sightline_clear(camera_pos, target, base_height, margin_m: float = 0.02, samples: int = 25) -> bool:
    """The camera's line of sight to `target` misses the Go2 body proxy (the last 3 cm are not checked)."""
    camera_pos, target = np.asarray(camera_pos, float), np.asarray(target, float)
    length = float(np.linalg.norm(target - camera_pos))
    for t in np.linspace(0.0, max(0.0, 1.0 - 0.03 / max(length, 1e-6)), samples):
        point = camera_pos + t * (target - camera_pos)
        if all(low - margin_m < point[i] < high + margin_m for i, (low, high) in enumerate(BODY_BOX_B)):
            return False
    return True


def plan_observation(joints, links, model: CameraModel, mount: WristMount, look_at_b, up, q_start, base_height,
                     distances=(0.35, 0.40, 0.30, 0.45), elevations=(65.0, 55.0, 45.0, 75.0),
                     rolls=(0.0, 15.0, -15.0), wanted_margin_m: float = 0.04,
                     image_row_fraction: float = 0.3) -> ObservationPlan:
    """An arm pose whose wrist camera looks at `look_at_b` from a distance its depth can measure.

    The look point is placed `image_row_fraction` of the way down the image, not at its centre: the
    gripper fills the lower ~40% of a wrist camera mounted behind it, and a cup that lands there is hidden
    and not detected (Week 1 log, 2026-09-17: a cup 8 cm off-centre went unseen from all six viewpoints).

    Takes the first candidate with `wanted_margin_m` of clearance from the body proxy, else the one with
    the most: a view that only just clears the head leaves no clear path onwards to a grasp.
    """
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    look_at_b = np.asarray(look_at_b, dtype=float)
    heading = heading_to(look_at_b, up)
    mount_inverse = invert(mount.pose)
    # Pitch the optical axis down by the angle between the image centre and the wanted row.
    dip = math.atan((model.cy - image_row_fraction * model.height) / model.fy)
    tried, best = 0, None
    for distance in distances:
        if distance < model.min_depth_m + 0.08 or distance > model.max_depth_m:
            continue
        for elevation in elevations:
            e = math.radians(elevation)
            view = math.cos(e) * heading - math.sin(e) * up
            cam_pos = look_at_b - distance * view
            if not sightline_clear(cam_pos, look_at_b, base_height):
                continue
            for roll in rolls:
                tried += 1
                optical = optical_rotation(view, up, roll)
                x, y, z = optical.T
                optical = np.column_stack([x, math.cos(dip) * y - math.sin(dip) * z, math.sin(dip) * y + math.cos(dip) * z])
                target = transform(optical, cam_pos) @ mount_inverse
                result = solve_link6(joints, links, target[:3, 3], target[:3, :3], _seeds(target[:3, 3], [q_start]),
                                     base_height, up, offset=(0.0, 0.0, 0.0))
                if result is None:
                    continue
                if not path_clear(joints, q_start, result.q, base_height, up)[0]:
                    continue
                pose = transform(optical, cam_pos)
                plan = ObservationPlan(result.q, pose, look_at_b, distance, elevation, roll)
                margin = proxy_margin(joints, result.q, base_height, up)
                if margin >= wanted_margin_m:
                    return plan
                if best is None or margin > best[0]:
                    best = (margin, plan)
    if best is not None:
        return best[1]
    raise PlanningError(f"no viewpoint of {np.round(look_at_b, 3).tolist()} from {tried} candidates "
                        f"(min depth {model.min_depth_m:.2f} m)")


def floor_point(x: float, y: float, up, base_height: float) -> np.ndarray:
    """A point on the floor in the base frame: (x, y) along the base axes projected level, at -base_height."""
    up, _, _ = horizontal_basis(up)
    point = np.array([x, y, 0.0])
    point -= up * (point @ up)
    return point - up * base_height


__all__ = ["GraspParams", "GraspPlan", "ObservationPlan", "PlanningError", "JAW_CENTRE_LINK6", "GRIPPER_OPEN_M",
           "GRIPPER_CLOSED_M", "grip_travel_m", "plan_top_down_grasp", "plan_observation", "line_waypoints", "top_down_rotation",
           "jaw_positions", "floor_point", "sightline_clear"]

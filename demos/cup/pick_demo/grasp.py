"""Top-down cup grasps and camera viewpoints for the welded D1, planned with `d1_ik` (numpy only).

Why top-down. The pick starts with the Go2 lying on the floor, and the arm cannot bring a *level*
gripper down to a floor-standing cup from there: sampling 300k configurations inside the soft limits,
none puts the jaw centre within 8 cm of the floor with the approach axis within 15 deg of horizontal, at
any base height from 0.12 m (lying) to 0.27 m (standing). Pointing down, the jaws reach the upper part
of a ~10 cm cup 35-45 cm ahead of the base (Week 1 log, 2026-09-17). That fixes the grasp: fingers
straddle the cup body from above, jaw axis across the robot-to-cup line so a handle pointing away from
or towards the robot stays clear of both fingers.

A cup too wide for that is grasped by its wall instead (`GraspParams.wall_grasp`). The jaws open to a
fixed 77.2 mm, so anything wider has nothing the fingers can straddle; what is left is the wall itself.
`pinch` puts one finger inside the cup and one outside and closes on the wall, the way a person picks up
a wide mug. `inside_out` puts both shut fingers into the mouth and opens them against the inside of the
wall -- their outer faces span 39.2 to 99.2 mm, so a mouth of roughly 50-100 mm can be held that way.
Which one a cup gets is decided by the gripper it is being planned for, not by preference: the pinch
needs jaws that shut below a cup wall, which the real arm does (`pinch_closed_gap_m`, 2 mm, stated on
2026-09-17) and the URDF's do not -- they bottom out 17.2 mm apart (F-059), so a simulated pick falls
through to the inside-out grasp. Neither has yet been tried on a robot, in simulation or on the bench.

Nothing here knows where the floor is, and nothing asks. Everything -- the survey pose, the headings a
search sweeps through, the grasp -- is placed relative to the arm's own base, and the cup is measured by
the camera in that same frame. The clearance proxy is the Go2's trunk alone. What used to keep the arm
off the surface was a plane at a stated height, and a stated height is what nothing on this robot
measures: on the bench it was an operator's number that moved three times in a day, and where it was
wrong it either refused reachable cups or put the floor below the table. The fingertips go
`GraspParams.finger_overlap_m` (35 mm) below the rim of whatever the camera measured, so on a cup at
least that deep they stop inside the cup, above its base, wherever that cup happens to stand. That
assumption -- cups at least 35 mm deep, stated by Lukas on 2026-09-17 -- is what replaces the floor.

Everything is in the Go2 base frame. The gripper geometry is the URDF's CAD (`workspace.tip_offsets`),
not a measurement of the real gripper.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

import d1_ik
from position_only.workspace import (BODY_BOX_B, EFFORT_LIMIT_NM, LINK_RADIUS_M, MOUNT_B, SOFT_LIMIT_FACTOR,
                                     clear_of_body, forward, gravity_torques)

from .camera import CameraModel, WristMount, camera_pose, invert, pixel_ray, transform
from .perception import CupEstimate

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

# Distal finger geometry in the Link6 frame, from the same CAD meshes (`workspace.gripper_points`, zero
# travel). Below the palm each finger spans |y| = 8.6 mm (the pad's inner face) to 19.6 mm (its outer
# face, at the tip block) and x = -12.6 to +13.4 mm; above z = 8.6 cm it widens to |y| = 29.6 mm, so only
# the last 3.9 cm of finger is narrow enough to go inside a cup at all. Reaching inside, it is the
# *outer* faces that meet the wall, so an inside-out grasp's span is 39.2 mm with the fingers shut and
# 99.2 mm fully open -- not the 17.2-77.2 mm jaw gap, which is what an outside grasp works to.
FINGER_INNER_Y_M = 0.0086
FINGER_OUTER_Y_M = 0.0196
FINGER_X_RANGE_M = (-0.0126, 0.0134)
FINGER_SHOULDER_Z_M = 0.086
CLOSED_SPAN_M = 2 * FINGER_OUTER_Y_M
OPEN_SPAN_M = CLOSED_SPAN_M + 2 * GRIPPER_OPEN_M
MAX_INSERT_M = FINGERTIP_Z_M - FINGER_SHOULDER_Z_M

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
    # The narrowest gap a finger may leave beside the cup at the worst point of the descent. The open jaws
    # are a fixed 77.2 mm, so a wide cup leaves little room: with a 70 mm cup there are 3.6 mm a side, less
    # than `line_tolerance_m`, and a descent that wanders its full tolerance would strike the rim. Rather
    # than refuse every cup over a fixed width, the descent is planned to whatever tolerance the clearance
    # actually allows (`descent_tolerance_for`), and only a cup that leaves no room at all is refused.
    jaw_safety_m: float = 0.001
    # How finely a straight descent may be split. A wide cup buys its clearance with a tighter tolerance,
    # and a tighter tolerance needs more segments: the 18 cm descent onto a 70 mm cup wants 36, which the
    # old cap of 32 refused one step short (worst 2.7 mm against 2.6 mm allowed). A narrow cup never comes
    # near this, so raising it changes nothing for the cups already planned.
    line_max_segments: int = 64
    # --- wall grasps: what to do when the cup is too wide for the jaws to take from the outside ---
    # "auto" tries the outside grasp, then inside-out, then the wall pinch; "off" is the outside grasp
    # alone (what the planner did before); "wall", "inside_out" and "pinch" force one.
    wall_grasp: str = "auto"
    # Estimated, not measured: perception fits one circle to the rim and never sees the wall's thickness.
    # 5 mm is a ceramic mug. It sets both how far inside the rim circle the inside is taken to be and,
    # for a pinch, how thin the thing between the pads is.
    wall_thickness_m: float = 0.005
    wall_insert_m: float = 0.030         # fingertips this far below the rim, subject to the limits in `insert_depth_m`
    wall_min_insert_m: float = 0.012     # less finger than this inside the cup is not worth attempting
    wall_floor_clearance_m: float = 0.020    # ...and they stop this far above the inside of the cup's base
    # Span commanded past the widest the inside can be, so the drives press on the wall rather than stop
    # at it -- the outward counterpart of `squeeze_m`, on the same unverified hardware scale. It sits on
    # top of the wall-thickness uncertainty rather than replacing it: see `_inside_out_recipe`.
    wall_press_m: float = 0.004
    # What the jaws really shut to at zero travel, which decides whether a pinch can touch a cup wall at
    # all. 2 mm is **stated, not measured**: the arm was watched closing "almost fully" under command on
    # 2026-09-17, and the probe behind F-063 saw the pads touch at servo 6's -19.7, which zero travel now
    # maps to. Two millimetres is the conservative reading of that, and only a wall thinner than it would
    # be refused by the difference. It is not the simulator's gripper -- the URDF jaws really do stop
    # 17.2 mm apart (F-059) and pinch nothing, so a simulated run passes `CLOSED_GAP_M` here
    # (`run_pick_demo.py --pinch_closed_gap`). Replace the 2 mm with a gauge reading when one exists.
    pinch_closed_gap_m: float = 0.002


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
    mode: str = "outside"                        # "outside", "inside_out" or "pinch"
    gripper_descend_m: float = GRIPPER_OPEN_M    # per-finger travel held through the approach and the descent
    gripper_grasp_m: float = GRIPPER_CLOSED_M    # ...and commanded once the fingers are at the cup

    def as_dict(self) -> dict:
        r = lambda a: [round(float(v), 4) for v in np.asarray(a).ravel()]
        return {"mode": self.mode, "heading_b": r(self.heading_b), "tilt_deg": self.tilt_deg,
                "jaw_pregrasp_b": r(self.jaw_pregrasp_b), "jaw_grasp_b": r(self.jaw_grasp_b),
                "jaw_lift_b": r(self.jaw_lift_b), "q_pregrasp": r(self.q_pregrasp),
                "approach_waypoints": len(self.approach), "descend_waypoints": len(self.descend),
                "lift_waypoints": len(self.lift),
                "gripper_descend_m": round(float(self.gripper_descend_m), 4),
                "gripper_grasp_m": round(float(self.gripper_grasp_m), 4), "notes": self.notes}


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


def arm_clear(joints, qs) -> np.ndarray:
    """`workspace.clear_of_body`: the arm's links clear of the Go2's trunk box, and nothing else.

    There is no ground test. It used to be a plane at z = -base_height in the base frame, tilted to
    gravity, and the height came from the simulator's root pose or an operator typing a number. A number
    that is wrong does not fail loudly: 3 cm too high refused a mug the camera could see, and 9 cm too low
    put the "floor" under the bench's table (Week 1 log, 2026-09-17). What keeps the fingers out of the
    surface now is the grasp itself -- they stop `finger_overlap_m` below a rim the camera measured, so
    inside any cup at least that deep.
    """
    return clear_of_body(joints, np.asarray(qs, dtype=float).reshape(-1, 6), math.inf)


def path_clear(joints, q_start, q_goal):
    """`d1_ik.path_clearance` with `arm_clear`: (clear, first failing fraction or None)."""
    configs = d1_ik.traversal_configs(q_start, q_goal)
    ok = arm_clear(joints, configs)
    if bool(ok.all()):
        return True, None
    return False, float(np.flatnonzero(~ok)[0]) / max(1, len(ok) - 1)


def proxy_margin(joints, q) -> float:
    """How far (m) the arm is from failing `arm_clear`: the smallest gap between its link segments (same
    chain and radius as the proxy) and the body box. Negative inside."""
    points = _chain_points(joints, q)[0]
    low, high = np.array(BODY_BOX_B)[:, 0], np.array(BODY_BOX_B)[:, 1]
    outside = np.linalg.norm(np.maximum(np.maximum(low - points, points - high), 0.0), axis=1)
    inside = np.minimum(points - low, high - points).min(axis=1)
    box = np.where(outside > 0.0, outside, -np.maximum(inside, 0.0))
    return float((box - LINK_RADIUS_M).min())


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


def solve_link6(joints, links, pos, rot, seeds, offset=JAW_CENTRE_LINK6):
    """First seed that converges, clears the trunk proxy (`arm_clear`) and can be held against gravity."""
    for seed in seeds:
        result = d1_ik.solve(joints, pos, rot, q0=seed, body="Link6", offset=offset, max_iterations=300)
        if (result.converged and bool(arm_clear(joints, result.q)[0])
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


def line_waypoints(joints, links, q_start, jaw_start, jaw_end, rot, tolerance_m: float = 0.004,
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
            result = solve_link6(joints, links, point, rot, [previous, *_seeds(point)])
            if result is None:
                raise PlanningError(f"no IK solution {100 * i / segments:.0f}% along the line to "
                                    f"{np.round(jaw_end, 3).tolist()}")
            configs = d1_ik.traversal_configs(previous, result.q)
            worst = max(worst, float(line_deviation(jaw_positions(joints, configs), jaw_start, jaw_end).max()))
            clear, _ = path_clear(joints, previous, result.q)
            if not clear:
                raise PlanningError("a straight-line segment enters the trunk proxy")
            if worst > tolerance_m:
                break
            waypoints.append(result.q)
            previous = result.q
        else:
            return waypoints
        segments *= 2
    raise PlanningError(f"straight line needs more than {max_segments} segments to stay within "
                        f"{1000 * tolerance_m:.0f} mm (worst {1000 * worst:.1f} mm)")


def jaw_clearance_per_side_m(radius_m: float) -> float:
    """Room between one open finger and the cup wall, with the jaw centre on the cup axis.

    The jaws open to a fixed 77.2 mm whatever the cup is, so this is simply how much of that the cup does
    not use. It is the budget the descent has to stay inside.
    """
    return float(OPEN_GAP_M / 2.0 - radius_m)


def descent_tolerance_for(radius_m: float, params: "GraspParams") -> float:
    """How far the jaw centre may stray from the descent line for a cup of this radius.

    `line_tolerance_m` is what the planner would like; the clearance beside the cup is what it can have.
    A 55 mm cup leaves 11.1 mm a side and the tolerance is unaffected; a 70 mm cup leaves 3.6 mm, so the
    descent is split more finely until it holds to 2.6 mm instead. Zero or less means the cup fills the
    jaws and no descent, however fine, keeps the fingers off it.
    """
    return min(params.line_tolerance_m, jaw_clearance_per_side_m(radius_m) - params.jaw_safety_m)


def inside_radius_m(cup: CupEstimate, params: "GraspParams") -> float:
    """What the planner takes the *inside* of the cup to be, for a grasp that reaches into it.

    Perception fits one circle to the rim, to points lying anywhere between the inner and outer edges of
    the lip, and never sees how thick the wall is. Taking a whole assumed wall thickness off that circle
    therefore under-reads the inside rather than over-reads it: the fingers go in with room to spare. It
    is the wrong end of the range to press against, though, which is why `_inside_out_recipe` opens the
    fingers to `radius_m` itself and not to this.
    """
    return float(cup.radius_m - params.wall_thickness_m)


def finger_corners_link6(travel_m: float, insert_m: float) -> np.ndarray:
    """Corners of the part of the fingers that goes inside a cup, in Link6, at gripper travel `travel_m`.

    Each distal finger is taken as a box: `FINGER_X_RANGE_M` across, |y| from the pad's inner face to its
    outer face (both carried outwards by the travel), and the last `insert_m` of its length. Above
    `FINGER_SHOULDER_Z_M` the finger widens to |y| = 29.6 mm, so nothing deeper than `MAX_INSERT_M` is
    described by this box and the insert is capped there.
    """
    insert = min(float(insert_m), MAX_INSERT_M)
    zs = (FINGERTIP_Z_M - insert, FINGERTIP_Z_M)
    ys = (FINGER_INNER_Y_M + travel_m, FINGER_OUTER_Y_M + travel_m)
    return np.array([[x, s * y, z] for x in FINGER_X_RANGE_M for y in ys for s in (-1.0, 1.0) for z in zs])


def finger_reach_m(rot, up, travel_m: float, insert_m: float) -> float:
    """How far from the cup's axis the inserted fingers reach, for a gripper held in `rot`.

    The fingertip centre is put on the cup's axis, so this is the largest horizontal distance from that
    axis any corner of the inserted part of either finger reaches -- the radius the cup's inside has to
    clear. It is computed from the corners rather than from |y| alone because the fingers are 26 mm wide
    across the jaw axis and, at a tilt, lean into the wall as well.
    """
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    points = (np.asarray(rot, dtype=float)
              @ (finger_corners_link6(travel_m, insert_m) - np.array([0.0, 0.0, FINGERTIP_Z_M])).T).T
    horizontal = points - np.outer(points @ up, up)
    return float(np.linalg.norm(horizontal, axis=1).max())


def inside_out_travel_m(rot, up, radius_m: float, insert_m: float) -> float | None:
    """Least gripper travel whose inserted fingers reach `radius_m` from the cup's axis.

    `finger_reach_m` grows with the travel, so it is found by bisection. None if the fingers cannot reach
    that far even fully open.
    """
    if finger_reach_m(rot, up, GRIPPER_OPEN_M, insert_m) < radius_m:
        return None
    low, high = GRIPPER_CLOSED_M, GRIPPER_OPEN_M
    for _ in range(40):
        mid = (low + high) / 2.0
        if finger_reach_m(rot, up, mid, insert_m) < radius_m:
            low = mid
        else:
            high = mid
    return float(high)


def insert_depth_m(cup: CupEstimate, params: "GraspParams", tilt_deg: float):
    """How far below the rim the fingertips may go for a wall grasp; a string saying why not, if too little.

    Three limits: the wrist shell must stay clear of the rim, as for an outside grasp; only the last
    `MAX_INSERT_M` of finger is narrow enough to be inside a cup; and the fingers stop short of the inside
    of the cup's base. The last of those rests on the estimated cup *height*, which a short-arc look can
    get badly wrong (F-058 reported a mug as 157 mm tall), so a wrong height here shortens or refuses the
    insertion rather than driving the fingers into the bottom -- unless it reads tall, when it does not.
    """
    insert = min(params.wall_insert_m,
                 shell_height_above_tips(tilt_deg) - params.rim_clearance_m,
                 MAX_INSERT_M,
                 cup.height_m - params.wall_floor_clearance_m)
    if insert < params.wall_min_insert_m:
        return (f"only {1000 * insert:.0f} mm of finger would be inside the cup, less than the "
                f"{1000 * params.wall_min_insert_m:.0f} mm worth attempting")
    return float(insert)


@dataclass
class _Recipe:
    """What one grasp mode asks for at one tilt: where the fingertips go, and what the gripper does there."""

    mode: str
    insert_m: float          # fingertips this far below the rim
    offsets_m: tuple         # jaw centre offsets from the cup's axis along the jaw axis, most wanted first
    tolerance_m: float       # how far the jaw centre may stray from the descent line
    gripper_descend_m: float
    gripper_grasp_m: float
    note: str


def _outside_recipe(cup, params, rot, up, tilt_deg):
    """Fingers straddling the cup's body from above: the grasp every recorded pick has used."""
    overlap = min(params.finger_overlap_m, shell_height_above_tips(tilt_deg) - params.rim_clearance_m)
    if overlap < params.min_overlap_m:
        return f"the wrist shell would leave only {1000 * overlap:.0f} mm of grip"
    clearance = jaw_clearance_per_side_m(cup.radius_m)
    tolerance = descent_tolerance_for(cup.radius_m, params)
    if tolerance <= 0.0:
        return (f"cup is {200 * cup.radius_m:.1f} cm wide; the open jaws are {100 * OPEN_GAP_M:.1f} cm, "
                f"leaving {1000 * clearance:.1f} mm a side -- no descent keeps the fingers off it")
    return _Recipe("outside", overlap, (0.0,), tolerance, GRIPPER_OPEN_M,
                   grip_travel_m(2.0 * cup.radius_m, params.squeeze_m),
                   f"fingertips {1000 * overlap:.0f} mm below the rim; {1000 * clearance:.1f} mm a side "
                   f"beside the cup")


def _inside_out_recipe(cup, params, rot, up, tilt_deg):
    """Both fingers into the mouth with the jaws shut, then opened against the wall from inside.

    For a cup too wide to fit between the open jaws this is the grasp the *modelled* gripper can still
    make: the fingers' outer faces span 39.2 mm shut and 99.2 mm open, so a mouth anything from about
    50 mm to 100 mm across can be held from within. It grips by pressing outwards on two opposite patches
    of wall, which is force closure by friction and self-centring, and it does not care where the handle
    is. It does care that the cup is a cup: a full one, or one light enough to be dragged rather than
    gripped, is not modelled here, and neither is the wall flexing.
    """
    insert = insert_depth_m(cup, params, tilt_deg)
    if isinstance(insert, str):
        return insert
    inner = inside_radius_m(cup, params)
    shut = finger_reach_m(rot, up, GRIPPER_CLOSED_M, insert)
    room = inner - shut
    if room <= params.jaw_safety_m:
        return (f"the shut fingers reach {2000 * shut:.0f} mm across and the mouth is "
                f"{2000 * inner:.0f} mm: they do not go in")
    if inside_out_travel_m(rot, up, inner, insert) is None:
        return (f"the mouth is {2000 * inner:.0f} mm across and the fingers reach "
                f"{2000 * finger_reach_m(rot, up, GRIPPER_OPEN_M, insert):.0f} mm fully open: "
                f"they cannot press on the wall")
    # Press to the *far* end of where the wall can be, not the near one. The rim circle is fitted to
    # points anywhere between the lip's inner and outer edges, so the inside is somewhere in
    # [radius - wall, radius]: the descent has to assume the near end (above), and the press the far one
    # plus a margin. Opening to the near end is what a simulated 90 mm cup showed to be worthless -- the
    # fingers stopped 1.0 mm short of a wall they had been driven 30 mm to reach, and the arm lifted away
    # from a cup it had never touched (Week 1 log, 2026-09-17). Over-travel costs little by comparison:
    # the drives are force limited (15 N), so fingers commanded past the wall press on it.
    press_to = cup.radius_m + params.wall_press_m / 2.0
    travel = inside_out_travel_m(rot, up, press_to, insert) or GRIPPER_OPEN_M
    return _Recipe("inside_out", insert, (0.0,), min(params.line_tolerance_m, room - params.jaw_safety_m),
                   GRIPPER_CLOSED_M, travel,
                   f"inside-out: shut fingers {1000 * insert:.0f} mm into a mouth of "
                   f"{2000 * inner:.0f}-{2000 * cup.radius_m:.0f} mm with {1000 * room:.0f} mm a side, then "
                   f"opened to {1000 * travel:.1f} mm of travel, reaching "
                   f"{2000 * finger_reach_m(rot, up, travel, insert):.0f} mm across the inside")


def _pinch_recipe(cup, params, rot, up, tilt_deg):
    """One finger inside the cup and one outside, closing on the wall itself.

    This is the grasp for a cup too wide even to reach into, and the one the modelled gripper cannot
    make: its pads bottom out 17.2 mm apart and a mug wall is 5-8 mm, so closing on the wall never
    touches it (F-059). Lukas backdrove the powered-down gripper until the pincers met, so the real rails
    go further than the URDF's, and on 2026-09-17 he had the arm closing almost fully under command.
    `pinch_closed_gap_m` is where that goes -- 2 mm, stated rather than measured -- and a planner given
    the URDF's 17.2 mm refuses here rather than planning a grasp known not to close.

    The jaw centre goes over the wall a quarter turn round the rim from the robot-to-cup line, where the
    jaw axis is already radial, so `top_down_rotation` is unchanged and a handle pointing at or away from
    the robot is as clear of the fingers as it is for an outside grasp. A handle *beside* the cup is not:
    perception does not report where it is.
    """
    insert = insert_depth_m(cup, params, tilt_deg)
    if isinstance(insert, str):
        return insert
    gap = float(params.pinch_closed_gap_m)
    shut = closed_travel_m(params)
    if gap >= params.wall_thickness_m:
        return (f"the jaws shut to {1000 * gap:.1f} mm and the wall is {1000 * params.wall_thickness_m:.0f} mm: "
                f"closing on it would never touch it (F-059; pinch_closed_gap_m is what the jaws shut to)")
    # Go down as wide as the cup allows, stopping only where the inner finger would reach the far wall.
    # A narrower gap would centre the jaw on the wall more neatly and shorten the closing sweep, but it
    # would also be an *intermediate* opening, and on the arm an intermediate opening is a guess: the
    # travel-to-gap relation there is not the CAD's and nobody has measured it (F-063). Wide open and
    # shut are the two commands that are calibrated, so a pinch uses only those (Lukas, 2026-09-17).
    mid = cup.radius_m - params.wall_thickness_m / 2.0
    inner = inside_radius_m(cup, params)
    room_to_far_wall = mid + inner - params.jaw_safety_m - FINGER_OUTER_Y_M
    descend_travel = float(np.clip(room_to_far_wall, shut, GRIPPER_OPEN_M))
    straddle = (jaw_gap_m(descend_travel) - params.wall_thickness_m) / 2.0
    if straddle <= params.jaw_safety_m:
        return (f"the jaws reach {1000 * jaw_gap_m(descend_travel):.0f} mm apart here, leaving "
                f"{1000 * straddle:.1f} mm either side of a {1000 * params.wall_thickness_m:.0f} mm wall")
    if descend_travel <= shut:
        return "the inner finger would reach the far wall before the jaws are open at all"
    return _Recipe("pinch", insert, (mid, -mid), min(params.line_tolerance_m, straddle - params.jaw_safety_m),
                   descend_travel, shut,
                   f"pinching the wall a quarter turn round the rim: one finger {1000 * insert:.0f} mm inside, "
                   f"one outside, going down {1000 * jaw_gap_m(descend_travel):.0f} mm apart "
                   f"({1000 * straddle:.0f} mm either side of the wall), then shut to {1000 * gap:.1f} mm "
                   f"on a {1000 * params.wall_thickness_m:.0f} mm wall")


_RECIPES = {"outside": _outside_recipe, "inside_out": _inside_out_recipe, "pinch": _pinch_recipe}
# Pinch before inside-out: on the real arm it is how a person takes a wide mug, and the jaws shut far
# enough for it (`pinch_closed_gap_m`). The order costs nothing where it cannot be done -- against the
# simulator's 17.2 mm jaws the pinch refuses on its first line and the inside-out grasp is planned instead.
_MODE_ORDER = {"auto": ("outside", "pinch", "inside_out"), "off": ("outside",), "outside": ("outside",),
               "wall": ("pinch", "inside_out"), "inside_out": ("inside_out",), "pinch": ("pinch",)}


def _attempt(joints, links, cup, params, up, heading, rot, tilt_deg, q_start, recipe, offset):
    """One recipe at one offset: a GraspPlan, or a string saying what stopped it."""
    approach_axis, jaw_axis = rot[:, 2], rot[:, 1]
    fingertip = cup.top_centre_b - up * recipe.insert_m + jaw_axis * offset
    jaw_grasp = fingertip - approach_axis * (FINGERTIP_Z_M - JAW_CENTRE_LINK6[2])
    jaw_pre = jaw_grasp - approach_axis * params.pregrasp_clearance_m
    jaw_lift = jaw_grasp + up * params.lift_m
    pre = solve_link6(joints, links, jaw_pre, rot, _seeds(jaw_pre, [q_start]))
    if pre is None:
        return "pregrasp unreachable"
    approach, notes = approach_path(joints, links, q_start, pre.q, jaw_pre, rot, up)
    if approach is None:
        return "every path to the pregrasp crosses the trunk proxy"
    try:
        descend = line_waypoints(joints, links, pre.q, jaw_pre, jaw_grasp, rot,
                                 recipe.tolerance_m, params.descend_step_m, params.line_max_segments)
        lift = line_waypoints(joints, links, descend[-1], jaw_grasp, jaw_lift, rot,
                              params.lift_tolerance_m, params.lift_step_m, params.line_max_segments)
    except PlanningError as exc:
        return str(exc)
    notes.append(recipe.note)
    notes.append(f"descent held to {1000 * recipe.tolerance_m:.1f} mm over {len(descend)} waypoints")
    return GraspPlan(heading, tilt_deg, rot, jaw_pre, jaw_grasp, jaw_lift, pre.q, approach, descend, lift,
                     notes, recipe.mode, recipe.gripper_descend_m, recipe.gripper_grasp_m)


def plan_top_down_grasp(joints, links, cup: CupEstimate, up, q_start,
                        params: GraspParams | None = None) -> GraspPlan:
    """The first top-down grasp of `cup` that plans: outside the cup if it fits, otherwise by its wall.

    Modes are tried in the order `params.wall_grasp` gives (`_MODE_ORDER`), each across every tilt, so a
    cup the jaws can take from the outside is planned exactly as it was before this fallback existed. A
    cup too wide for that is taken from inside instead, and the refusal, when everything fails, carries
    every mode's reason with its numbers.

    Everything here is placed from `cup`, which the camera measured in this frame, and the surface the cup
    stands on is never named. An outside grasp puts the fingertips `finger_overlap_m` below the rim and a
    wall grasp less than that (`insert_depth_m`), so on a cup at least 35 mm deep the tips stop inside it.
    A shallower cup -- a saucer, a tray -- would have them reach past its base and into whatever it stands
    on, and nothing in this module would refuse that.
    """
    params = params or GraspParams()
    if params.finger_overlap_m >= FINGERTIP_Z_M - PALM_Z_M:
        raise PlanningError("finger overlap would put the palm on the rim")
    if params.wall_grasp not in _MODE_ORDER:
        raise PlanningError(f"unknown wall_grasp {params.wall_grasp!r}; expected one of "
                            f"{', '.join(sorted(_MODE_ORDER))}")
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    heading = heading_to(cup.top_centre_b, up)
    reasons = []
    for mode in _MODE_ORDER[params.wall_grasp]:
        for tilt in params.tilts_deg:
            rot = top_down_rotation(heading, up, tilt)
            recipe = _RECIPES[mode](cup, params, rot, up, tilt)
            if isinstance(recipe, str):
                reasons.append((mode, tilt, recipe))
                continue
            for offset in recipe.offsets_m:
                outcome = _attempt(joints, links, cup, params, up, heading, rot, tilt, q_start, recipe, offset)
                if isinstance(outcome, GraspPlan):
                    return outcome
                reasons.append((mode, tilt, outcome))
    raise PlanningError(_refusal(reasons))


def _refusal(reasons) -> str:
    """One line per distinct reason, naming the tilts it happened at.

    Three modes across four tilts is twelve refusals, and a cup that is simply too wide gives the same
    sentence every time. The tilts stay in it because which tilts failed is the difference between a cup
    that cannot be grasped and one that cannot be reached.
    """
    grouped = {}
    for mode, tilt, text in reasons:
        grouped.setdefault((mode, text), []).append(f"{tilt:.0f}")
    return "; ".join(f"{mode} tilt {'/'.join(tilts)}: {text}" for (mode, text), tilts in grouped.items())


def shell_height_above_tips(tilt_deg: float) -> float:
    """How far above the fingertips (along up) the lowest point of the Link6 shell sits, for a top-down grasp.

    Tilting leans the approach axis away from the robot, which lowers the shell's robot-side edge (+x in
    Link6) by its offset times sin(tilt). Straight down that is the 4.9 cm of finger below the palm.
    """
    tilt = math.radians(tilt_deg)
    return (FINGERTIP_Z_M - PALM_Z_M) * math.cos(tilt) - max(PALM_X_RANGE_M[1] * math.sin(tilt),
                                                               PALM_X_RANGE_M[0] * math.sin(tilt))


def jaw_gap_m(travel_m: float) -> float:
    """The gap between the pads' inner faces at a per-finger travel, from the CAD: 17.2 mm at zero.

    Travel is allowed to go *below* zero. The URDF stops each finger there, but that stop is the CAD's,
    not the arm's: the real pads meet (F-063), which is 8.6 mm of travel further in than the model has,
    and a wall pinch lives entirely in that region. `closed_travel_m` is where a given gripper runs out.
    """
    return CLOSED_GAP_M + 2.0 * travel_m


def closed_travel_m(params: "GraspParams") -> float:
    """Per-finger travel at which the jaws are shut, for the gripper `params` describes.

    Zero for the CAD gripper, whose pads stop 17.2 mm apart; about -7.6 mm for the real arm, whose pads
    come to within `pinch_closed_gap_m`. Negative travel is not a trick: it is the same travel axis
    carried past the limit the URDF import happened to record.
    """
    return float(min(0.0, (params.pinch_closed_gap_m - CLOSED_GAP_M) / 2.0))


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


def approach_path(joints, links, q_start, q_goal, jaw_goal, rot, up, raises=(0.08, 0.16)):
    """Joint waypoints ending at `q_goal` whose every move clears the proxy, and notes; (None, notes) if none.

    Tries the direct move, then a via pose above the goal, then one above where the arm starts (backing
    out of a low start before swinging over), then both.
    """
    if path_clear(joints, q_start, q_goal)[0]:
        return [q_goal], []
    frames, _, _ = forward(joints, np.asarray(q_start, dtype=float).reshape(1, 6))
    start_rot, start_pos = frames["Link6"][0][0], frames["Link6"][1][0]
    start_jaw = start_pos + start_rot @ np.asarray(JAW_CENTRE_LINK6)
    candidates = []
    for rise in raises:
        above_goal = solve_link6(joints, links, jaw_goal + up * rise, rot, _seeds(jaw_goal, [q_goal, q_start]))
        above_start = solve_link6(joints, links, start_jaw + up * rise, start_rot, [q_start])
        if above_goal is not None:
            candidates.append(([above_goal.q, q_goal], f"via a pose {100 * rise:.0f} cm above the pregrasp"))
        if above_start is not None:
            candidates.append(([above_start.q, q_goal], f"backing {100 * rise:.0f} cm up out of the start pose"))
        if above_goal is not None and above_start is not None:
            candidates.append(([above_start.q, above_goal.q, q_goal], f"up {100 * rise:.0f} cm, across, and down"))
    for waypoints, note in candidates:
        previous, ok = q_start, True
        for q in waypoints:
            if not path_clear(joints, previous, q)[0]:
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


def sightline_clear(camera_pos, target, margin_m: float = 0.02, samples: int = 25) -> bool:
    """The camera's line of sight to `target` misses the Go2 body proxy (the last 3 cm are not checked)."""
    camera_pos, target = np.asarray(camera_pos, float), np.asarray(target, float)
    length = float(np.linalg.norm(target - camera_pos))
    for t in np.linspace(0.0, max(0.0, 1.0 - 0.03 / max(length, 1e-6)), samples):
        point = camera_pos + t * (target - camera_pos)
        if all(low - margin_m < point[i] < high + margin_m for i, (low, high) in enumerate(BODY_BOX_B)):
            return False
    return True


def gripper_ahead_of_body(joints, q, heading, up) -> bool:
    """The jaw centre is past the front of the trunk along `heading`, rather than hovering over it.

    The camera sits behind the fingers, so a viewpoint chosen only by where the *camera* has to be leaves
    the gripper wherever the arm can reach it from -- and with the mount at the back of the Go2 that is
    over the dog's own back, jaws pointing down at its head. Measured against a cup 42 cm ahead, every
    elevation up to 75 deg puts the jaw centre at x <= 0.29 m against a body box ending at 0.30 m; only a
    near-vertical look gets it out in front (Week 1 log, 2026-09-17).
    """
    jaw = jaw_positions(joints, q)[0]
    heading = np.asarray(heading, dtype=float)
    corners = np.array([[x, y, z] for x in BODY_BOX_B[0] for y in BODY_BOX_B[1] for z in BODY_BOX_B[2]])
    return bool(jaw @ heading > (corners @ heading).max())


@dataclass
class SurveyPlan(ObservationPlan):
    """An observation pose described by where the camera stands and how far it is pitched down."""

    pitch_deg: float = 0.0
    height_m: float = 0.0            # camera above the arm's mount, along up
    pivot_deg: float = 0.0           # where the centre of the view points, left of the survey's heading (`plan_pivot`)

    def as_dict(self) -> dict:
        d = super().as_dict()
        d.update(pitch_deg=round(self.pitch_deg, 1), camera_above_mount_m=round(self.height_m, 3),
                 pivot_deg=round(self.pivot_deg, 2))
        return d


def view_point(joints, q, mount, range_m: float):
    """The point `range_m` along the optical axis, and the camera pose it was taken from.

    Where a look is *aimed*, without asking what it lands on. The old version of this intersected the
    optical axis with the floor, which needed the floor's height; at the survey's pitch the two agree to
    within a couple of centimetres, and the only thing either is used for is a bearing about gravity.
    """
    pose = camera_pose(joints, q, mount)
    return pose[:3, 3] + pose[:3, 2] * float(range_m), pose


# `plan_survey` on the Go2, looking past its head. The pose the pick used before this one stood the camera
# 5 cm ahead of the arm's mount and 0.40 m above the floor, over the dog's back, and in simulation the head
# hid a cup 0.42 m straight ahead from it (Week 1 log, 2026-09-17). Scored on the CPU model for the pivoting search -- cups 0.35-0.50 m out at every
# 5 deg of bearing to +-45, whole cup in frame and clear of the fingers, sightlines to its middle and rim past
# the trunk proxy, the mount the simulator uses -- that pose sees 26 of 76 and this one 63: camera 20 cm ahead
# of the mount and 0.335 m above it, 60 deg down. The misses are all at 0.35-0.40 m, in front of the nose and
# past the corners of the head. The grid is ordered so that pose comes first; the rest are its neighbours in
# case it is out of reach. Not used on the bench, which has no dog.
#
# The heights are 0.50, 0.45 and 0.40 m above the floor as the pose was chosen (the Go2 settles lying with its
# base 0.0851 m up and 7.3 deg nose-up, so its mount is 0.0794 m along gravity from the base frame's origin),
# carried over here as heights above the mount so that nothing has to know the floor. On a dog that settles
# differently, they stay where they are relative to the arm and the frame moves over the floor instead.
SURVEY_PAST_THE_HEAD = {"pitches_deg": (60.0, 55.0, 65.0), "heights_m": (0.335, 0.285, 0.235),
                        "offsets_m": (0.20, 0.25), "see_past_body_m": 0.40, "sight_height_m": -0.115}


def plan_survey(joints, links, model: CameraModel, mount: WristMount, up, q_start, heading,
                pitches_deg=(45.0, 50.0, 55.0, 40.0, 60.0, 65.0), heights_m=(0.26, 0.31, 0.36, 0.41),
                offsets_m=(0.05, 0.15, 0.25, -0.05, 0.35), look_range_m: float = 0.60,
                wanted_margin_m: float = 0.04, max_candidates: int = 40,
                see_past_body_m: float | None = None, sight_height_m: float = -0.115) -> SurveyPlan:
    """One pose that looks out over the ground in front of the robot, angled rather than straight down.

    A survey pose is described the way a person would set one up, and entirely in the arm's own frame:
    stand the camera `height_m` above the arm's mount and `offset_m` ahead of it along `heading`, and tip
    it `pitch_deg` down from horizontal. What that lands on is the world's business. Looking down and
    forwards at an angle, the wrist is welcome to sit over the dog's own back -- Lukas, 2026-09-17, and it
    is where the height comes from -- so the search reaches back behind the shoulder as well as in front
    of it.

    The grid is tried in the order it is given and the first pose that is reachable, holdable, path-clear
    and `wanted_margin_m` clear of the body proxy is taken; failing that, the one with the most clearance.
    The order is the preference, so the pose a rig has actually used comes first in its grid. Poses used to
    be *scored* on how much of a stated cup region the frame held, which took the floor's height and a
    guess at where cups stand; both are gone. What remains is a cheap check that the frame points at the
    ground at all: the top row of the image must dip below horizontal, or the pose is looking out at the
    room.

    The defaults are the bench's, where the arm is bolted to the surface the cup stands on: the first pose
    they reach stands the camera 0.26 m above the mount, 5 cm ahead, 45 deg down, which is within a
    centimetre of the height the bench has used all along (2026-09-17). Over a table at about the mount's
    own level that frames 0.16 to 0.66 m out -- the whole of what this arm can reach -- and the grid runs
    upwards from there. A higher pose sees further but starts further out: from 0.41 m up, the near edge is
    0.43 m and the bench mug at 0.29 m would be behind the frame.

    The search stops after `max_candidates` poses in any case: an inverse-kinematics solve that fails costs
    every seed, and scoring a whole grid took 46 s while the robot sat waiting for it.

    Holding ground in the frame is not the same as seeing it. With the arm on the Go2, the pose the default
    grid picks -- camera 5 cm ahead of the mount, 45 deg down -- looks over the dog's head, and in
    simulation the head hid a cup 0.42 m straight ahead from every frame but its rim (Week 1 log,
    2026-09-17). `see_past_body_m` refuses a pose whose line of sight to a point that far along the heading,
    `sight_height_m` above the mount, crosses the trunk proxy (`sightline_clear`). The bare proxy agrees
    with that render -- from the old pose the rim's sightline passes 7.6 cm up at the trunk's front edge and
    the middle's 3.8 cm, against a 6 cm top -- and the check's 2 cm margin refuses both. None keeps the
    bench's behaviour, where the arm has no dog in front of it.

    `look_range_m` is how far along the optical axis the pose's `look_at_b` is taken to be. It names no
    surface: it is the range a pivot measures its bearings at (`plan_pivot`), and at the survey's pitch it
    is about where the axis meets the floor beside a lying Go2.
    """
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    heading = np.asarray(heading, dtype=float)
    heading = heading - up * (heading @ up)
    heading /= np.linalg.norm(heading)
    mount_inverse = invert(mount.pose)
    tried, best = 0, None
    for pitch in pitches_deg:
        rad = math.radians(pitch)
        view = math.cos(rad) * heading - math.sin(rad) * up
        optical = optical_rotation(view, up, 0.0)
        # The top row of the frame, which is the furthest it sees: pointing at or above horizontal, the
        # pose is looking out into the room rather than over the ground ahead.
        top_row = optical @ np.array([0.0, (0.0 - model.cy) / model.fy, 1.0])
        if top_row @ up >= -1e-6:
            continue
        for height in heights_m:
            for offset in offsets_m:
                if tried >= max_candidates and best is not None:
                    return best[1]
                tried += 1
                cam_pos = MOUNT_B + heading * offset + up * height
                if see_past_body_m is not None and not sightline_clear(
                        cam_pos, MOUNT_B + heading * see_past_body_m + up * sight_height_m):
                    continue
                target = transform(optical, cam_pos) @ mount_inverse
                # A short seed list on purpose: a failing solve pays for every seed it is given, and this
                # is a grid search where most candidates fail.
                result = solve_link6(joints, links, target[:3, 3], target[:3, :3],
                                     [np.asarray(q_start, dtype=float), *SEEDS], offset=(0.0, 0.0, 0.0))
                if result is None or not path_clear(joints, q_start, result.q)[0]:
                    continue
                margin = proxy_margin(joints, result.q)
                plan = SurveyPlan(result.q, transform(optical, cam_pos), cam_pos + view * look_range_m,
                                  float(look_range_m), pitch, 0.0, pitch_deg=pitch, height_m=height)
                if margin >= wanted_margin_m:
                    return plan
                if best is None or margin > best[0]:
                    best = (margin, plan)
    if best is None:
        raise PlanningError(f"no angled survey pose of the ground ahead from {tried} candidates")
    return best[1]


def pivot_stops_deg(model: CameraModel, sweep_deg: float) -> tuple:
    """Where a pivoting search points the view, in degrees left of straight ahead, in the order visited.

    Straight ahead first -- the survey pose itself -- then out to the left and back across to the right.
    Neighbouring stops are no further apart than half the image's horizontal field of view, so every
    heading in [-`sweep_deg`, +`sweep_deg`] falls in the middle half of some frame rather than at an edge,
    where a cup is cut off and its rim arc is short. The modelled D435 at 640x480 is 54.9 deg wide, which
    makes the stops 0, +-22.5 and +-45. Seen from the survey pose's pitch, a heading reaches further across
    the floor than across the image, so this is the conservative side of the spacing.
    """
    if sweep_deg <= 0.0:
        return (0.0,)
    fov = math.degrees(math.atan(model.cx / model.fx) + math.atan((model.width - model.cx) / model.fx))
    per_side = math.ceil(sweep_deg / (fov / 2.0) - 1e-9)
    left = [sweep_deg * i / per_side for i in range(1, per_side + 1)]
    return (0.0, *left, *(-a for a in left))


def _azimuth_deg(vector, heading, up) -> float:
    """Angle of `vector` about `up`, left of `heading` positive."""
    left = np.cross(up, heading)
    return math.degrees(math.atan2(float(vector @ left), float(vector @ heading)))


def _bearing_of(point, up, heading) -> float:
    """Bearing of `point` about gravity from the arm's mount, left of `heading` positive."""
    offset = np.asarray(point, dtype=float) - MOUNT_B
    return _azimuth_deg(offset - up * (offset @ up), heading, up)


def survey_heading(joints, mount, survey: SurveyPlan, up) -> np.ndarray:
    """Unit horizontal direction from the arm's mount to where the survey is looking: the zero that
    `plan_pivot` and `glimpse_bearing_deg` measure bearings from."""
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    aim, _ = view_point(joints, survey.q, mount, survey.distance_m)
    heading = aim - MOUNT_B
    heading -= up * (heading @ up)
    norm = float(np.linalg.norm(heading))
    if norm < 1e-6:
        raise PlanningError("the survey pose looks straight up or down; no heading to pivot about")
    return heading / norm


def glimpse_bearing_deg(joints, model: CameraModel, mount, q, pixel_uv, up, heading, range_m: float):
    """Bearing about gravity, left of `heading`, of whatever is at `pixel_uv` `range_m` from the camera.

    For steering a search, not for grasping, and it is why the range may be a nominal one: `q` is feedback,
    and while the arm swings it lags the camera that took the frame, so the bearing is already off by the
    swing's speed times that lag -- a larger error than the range's. The range is the survey's own
    `look_range_m`, and the parallax between the true range and that one is zero when the camera turns
    about gravity and a couple of degrees when the base lies tilted.
    """
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    pose = camera_pose(joints, q, mount)
    ray = pose[:3, :3] @ pixel_ray(model, float(pixel_uv[0]), float(pixel_uv[1]))
    return _bearing_of(pose[:3, 3] + ray * float(range_m), up, heading)


def plan_pivot(joints, links, model: CameraModel, mount: WristMount, survey: SurveyPlan, pivot_deg: float, up,
               q_start, tolerance_deg: float = 0.25, iterations: int = 8) -> SurveyPlan:
    """The survey pose turned on Joint1 alone, until the view lies `pivot_deg` left of the survey's.

    Only the base joint moves, so the camera keeps its height and its tilt down over the ground and the arm
    swings as one piece -- a search, not a re-plan. The angle is measured about gravity from the arm's
    mount, not read off Joint1: lying down the base settles 7.3 deg nose-up and Joint1's axis leans with
    it, so on the CPU model the view needs 47.3 deg of Joint1 to come 45 deg round (and the optical axis
    then points 41.3 deg round). Joint1 is solved for the view instead.

    "The view" is the point `survey.distance_m` along the optical axis (`view_point`), which is where the
    survey was aimed. It used to be where the axis met the floor, and the two differ by the difference
    between that range and the real one, which changes the bearing by a fraction of a degree.

    Refused, as a `PlanningError`, when the turn would need Joint1 past its soft limit, when the pose would
    fail the trunk proxy or could not be held against gravity, or when the swing from `q_start` crosses the
    proxy.
    """
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    heading = survey_heading(joints, mount, survey, up)
    low, high = joints["Joint1"]["limits"]
    middle, half = (low + high) / 2.0, (high - low) / 2.0 * SOFT_LIMIT_FACTOR
    q = np.asarray(survey.q, dtype=float).copy()
    q[0] = survey.q[0] + math.radians(pivot_deg)
    achieved = None
    for _ in range(iterations):
        q[0] = float(np.clip(q[0], middle - half, middle + half))
        aim, _ = view_point(joints, q, mount, survey.distance_m)
        achieved = _bearing_of(aim, up, heading)
        error = pivot_deg - achieved
        if abs(error) <= tolerance_deg:
            break
        q[0] += math.radians(error)
    else:
        raise PlanningError(f"pivot {pivot_deg:+.1f} deg: the view reaches {achieved:+.1f} deg with Joint1 at "
                            f"{q[0]:+.3f} rad (soft limit +-{half:.3f})")
    if not bool(arm_clear(joints, q)[0]):
        raise PlanningError(f"pivot {pivot_deg:+.1f} deg: the pose crosses the trunk proxy")
    if not holdable(joints, links, q):
        raise PlanningError(f"pivot {pivot_deg:+.1f} deg: the pose exceeds a joint's effort limit")
    clear, fraction = path_clear(joints, q_start, q)
    if not clear:
        raise PlanningError(f"pivot {pivot_deg:+.1f} deg: the swing there crosses the proxy {100 * fraction:.0f}% "
                            f"of the way")
    aim, pose = view_point(joints, q, mount, survey.distance_m)
    return SurveyPlan(q, pose, aim, survey.distance_m, survey.elevation_deg, 0.0,
                      pitch_deg=math.degrees(-math.asin(float(np.clip(pose[:3, 2] @ up, -1.0, 1.0)))),
                      height_m=float((pose[:3, 3] - MOUNT_B) @ up), pivot_deg=float(achieved))


__all__ = ["GraspParams", "GraspPlan", "ObservationPlan", "PlanningError", "JAW_CENTRE_LINK6", "GRIPPER_OPEN_M",
           "jaw_gap_m", "closed_travel_m",
           "GRIPPER_CLOSED_M", "grip_travel_m", "plan_top_down_grasp", "line_waypoints", "top_down_rotation",
           "jaw_positions", "sightline_clear", "gripper_ahead_of_body", "plan_survey", "view_point",
           "plan_pivot", "pivot_stops_deg", "SURVEY_PAST_THE_HEAD", "survey_heading", "glimpse_bearing_deg",
           "SurveyPlan", "inside_radius_m", "inside_out_travel_m",
           "finger_reach_m", "finger_corners_link6", "insert_depth_m", "CLOSED_SPAN_M", "OPEN_SPAN_M",
           "MAX_INSERT_M"]

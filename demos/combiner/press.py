"""Where the arm looks for the door's tag, and how it pushes the lever down, planned in numpy.

The lever is pushed, not grasped. The gripper comes in level, pointing at the door, jaws shut, and lays
both fingers across the top of the lever near its free end: the lever runs along the jaw axis, under the
fingers' flat undersides. Pushing it down is then a rigid rotation of that whole tool pose about the
spindle axis, which the arm follows through `final_deg`. Pushing needs no grasp to line up on an 18 mm
bar, tolerates error along the lever and along the door's normal, and the commanded pose sits a few
millimetres *inside* the lever so the fingers keep pressing on it.

Why this pose rather than fingertips from above. With the dog lying down the handle is 0.13 m above the
arm's mount and 0.45-0.55 m out. A near-vertical approach does not solve there, and a fingertip press
lands on 18 mm of bar with a 26 mm pad. On the CPU model the level press, the search stops and the close
look all solve at 300 of 300 seeded default placements (Week 1 log, 2026-09-19).

How hard it can push (`press_capacity`): the largest force along the push direction for which the torque
each joint needs -- holding the arm's own weight plus J^T F -- stays within the published limits
(`workspace.EFFORT_LIMIT_NM`). Only the direction and the joint limits enter, so it is a quasi-static
ceiling with the base level, not a prediction of what the servos' loops deliver. Pushing 80 mm out along
the lever it is 0.63-0.87 N·m at 45 degrees over those 300 placements (median 0.74): the elbow (Joint3)
binds while the lever is near level, the base yaw (Joint1) once the push has turned sideways.

Everything is in the Go2 base frame, from the box pose the tag gives (`apriltag.box_pose`).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

import d1_ik
from position_only.workspace import EFFORT_LIMIT_NM, MOUNT_B, forward, gravity_torques
from demos.cup.pick_demo.camera import invert, transform
from demos.cup.pick_demo.grasp import (FINGER_X_RANGE_M, FINGERTIP_Z_M, SEEDS, PlanningError, optical_rotation,
                                       path_clear, solve_link6)

from .apriltag import tag_pose_box
from .geometry import GEOMETRY


@dataclass(frozen=True)
class PressParams:
    radius_m: float = 0.08           # along the lever from the spindle axis, where the fingers cross it
    reach_past_m: float = 0.025      # the fingertips run this far past the lever's centreline, towards the door
    press_depth_m: float = 0.004     # commanded below the lever's top, so the fingers keep pressing
    final_deg: float = 52.0          # commanded lever angle at the end of the push; the latch needs 45
    clearance_m: float = 0.04        # above the lever before coming down onto it
    standoff_m: float = 0.08         # back from the lever, away from the door, before moving over it
    step_deg: float = 2.0            # arc waypoint spacing
    speed_deg_s: float = 15.0        # commanded lever speed while pushing
    hold_s: float = 2.0              # at the end of the push

    def contact_link6(self) -> np.ndarray:
        """The point on the fingers' underside that sits on the lever: Link6 +x is down when level."""
        return np.array([FINGER_X_RANGE_M[1], 0.0, FINGERTIP_Z_M - self.reach_past_m])


class Lever:
    """The lever's geometry in the base frame, from a box pose with the door closed."""

    def __init__(self, box_pose_b, geometry=GEOMETRY):
        self.box_pose = np.asarray(box_pose_b, dtype=float)
        self.g = geometry
        self.rot = self.box_pose[:3, :3]
        self.axis = self.rot @ np.array([1.0, 0.0, 0.0])      # spindle axis, out of the door
        self.pivot = self.rot @ np.asarray(geometry.lever_axis_point) + self.box_pose[:3, 3]

    def direction(self, deg: float) -> np.ndarray:
        a = math.radians(deg)
        return self.rot @ np.array([0.0, -math.cos(a), -math.sin(a)])

    def normal(self, deg: float) -> np.ndarray:
        """The lever's upper side's outward normal: up when it is level, turning with it."""
        a = math.radians(deg)
        return self.rot @ np.array([0.0, -math.sin(a), math.cos(a)])

    def top(self, deg: float, radius: float) -> np.ndarray:
        return self.pivot + radius * self.direction(deg) + self.g.handle_radius * self.normal(deg)

    def tool_rotation(self, deg: float) -> np.ndarray:
        """Link6: approach (z) into the door, jaw axis (y) along the lever, +x (finger undersides) into it."""
        return np.column_stack([-self.normal(deg), self.direction(deg), -self.axis])


def press_capacity(joints, links, q, point_b, push_b) -> dict:
    """The most the joint limits allow the arm to push at `point_b` along `push_b`, arm weight included.

    Static torque at each joint is gravity's (`workspace.gravity_torques`, base level) plus J^T F; the
    force is raised until the first joint reaches its published limit.
    """
    q = np.asarray(q, dtype=float).reshape(1, 6)
    _, axes, origins = forward(joints, q)
    jac = d1_ik.jacobian(axes, origins, np.asarray(point_b, dtype=float).reshape(1, 3))[0][:3]
    hold = gravity_torques(joints, links, q)[0]
    per_newton = jac.T @ (np.asarray(push_b, dtype=float) / np.linalg.norm(push_b))
    force, joint = math.inf, None
    for i, (a, b) in enumerate(zip(per_newton, hold)):
        if abs(a) < 1e-9:
            continue
        for limit in (EFFORT_LIMIT_NM[i], -EFFORT_LIMIT_NM[i]):
            f = (limit - b) / a
            if 0.0 < f < force:
                force, joint = f, f"Joint{i + 1}"
    return {"force_n": force, "limiting_joint": joint,
            "hold_torque_nm": [round(float(v), 3) for v in hold]}


def _solve(joints, links, pos, rot, seeds, offset):
    result = solve_link6(joints, links, pos, rot, seeds, offset=offset)
    return None if result is None else result.q


def _seeds(point_b, extra=()):
    yaw = math.atan2(float(point_b[1] - MOUNT_B[1]), float(point_b[0] - MOUNT_B[0]))
    out = [np.asarray(s, dtype=float) for s in extra if s is not None]
    for seed in SEEDS:
        candidate = seed.copy()
        candidate[0] = yaw
        out.append(candidate)
    return out


@dataclass
class PressPlan:
    standoff: np.ndarray
    approach: list                    # joint waypoints from the standoff to just above the lever
    contact: list                     # down onto the lever
    arc: list                         # (commanded lever deg, q), from 0 to `final_deg`
    capacity: list = field(default_factory=list)   # per arc angle: the static ceiling
    params: PressParams = field(default_factory=PressParams)

    def predicted_torque_nm(self, deg: float = GEOMETRY.handle_release_deg) -> float:
        """The static ceiling on the lever torque at the arc angle nearest `deg`."""
        row = min(self.capacity, key=lambda r: abs(r["lever_deg"] - deg))
        return row["handle_torque_nm"]

    def as_dict(self) -> dict:
        return {"params": {k: getattr(self.params, k) for k in self.params.__dataclass_fields__},
                "contact_link6_m": self.params.contact_link6().round(4).tolist(),
                "standoff_q": np.round(self.standoff, 4).tolist(),
                "arc": [{"lever_deg": d, "q": np.round(q, 4).tolist()} for d, q in self.arc],
                "static_capacity": self.capacity}


def _line(joints, links, lever, start, end, rot, offset, q_seed, steps: int):
    """IK waypoints carrying the contact point from `start` to `end` in `steps` straight segments."""
    qs = []
    for s in np.linspace(0.0, 1.0, steps + 1)[1:]:
        q = _solve(joints, links, (1 - s) * start + s * end, rot, _seeds(end, [q_seed]), offset)
        if q is None:
            return None
        qs.append(q)
        q_seed = q
    return qs


def plan_press(joints, links, box_pose_b, q_start, params: PressParams | None = None) -> PressPlan:
    """Joint waypoints for the whole push, or `PlanningError` naming the pose that does not solve."""
    p = params or PressParams()
    lever = Lever(box_pose_b)
    offset = p.contact_link6()
    rot0 = lever.tool_rotation(0.0)
    touch = lever.top(0.0, p.radius_m)
    above = touch + p.clearance_m * lever.normal(0.0)
    standoff = above + p.standoff_m * lever.axis
    q_standoff = _solve(joints, links, standoff, rot0, _seeds(standoff, [q_start]), offset)
    if q_standoff is None:
        raise PlanningError(f"no arm pose puts the fingers {100 * p.standoff_m:.0f} cm in front of the lever")
    if not path_clear(joints, q_start, q_standoff)[0]:
        raise PlanningError("the move to the lever's standoff crosses the trunk proxy")
    approach = _line(joints, links, lever, standoff, above, rot0, offset, q_standoff, 3)
    if approach is None:
        raise PlanningError("the fingers cannot come in over the lever")
    contact = _line(joints, links, lever, above, touch, rot0, offset, approach[-1], 2)
    if contact is None:
        raise PlanningError("the fingers cannot come down onto the lever")
    arc, capacity, q_prev = [], [], contact[-1]
    angles = list(np.arange(0.0, p.final_deg, p.step_deg)) + [p.final_deg]
    for deg in angles:
        deg = float(deg)
        target = lever.top(deg, p.radius_m) - p.press_depth_m * lever.normal(deg)
        q = _solve(joints, links, target, lever.tool_rotation(deg), _seeds(target, [q_prev]), offset)
        if q is None:
            raise PlanningError(f"the push does not solve at a lever angle of {deg:.0f} deg")
        arc.append((deg, q))
        q_prev = q
        ceiling = press_capacity(joints, links, q, lever.top(deg, p.radius_m), -lever.normal(deg))
        capacity.append({"lever_deg": round(deg, 1), "force_n": round(ceiling["force_n"], 2),
                         "handle_torque_nm": round(ceiling["force_n"] * p.radius_m, 3),
                         "limiting_joint": ceiling["limiting_joint"]})
    return PressPlan(q_standoff, approach, contact, arc, capacity, p)


# ------------------------------------------------------------------ looking for the tag

@dataclass(frozen=True)
class SearchParams:
    """Camera stops for finding the tag, placed from the arm's mount like the cup's survey.

    The lying dog's mount is ~0.17 m above the floor and the tag's centre 0.41 m, so the tag sits about
    0.24 m above the mount wherever the box is: the camera stands a little above that and looks at a point
    that high, `target_range_m` out, at each heading in turn. A dog standing up would need other numbers.
    """
    # Straight ahead, then out along one side and back across to the other. Alternating sides (0, +20, -20,
    # +40, ...) visits the stops in the same order of likelihood but swings Joint1 through 20, 40, 60, 80... deg
    # between them: 200 deg to reach -40 against 40 this way, about 2 s less on average over the six side stops.
    headings_deg: tuple = (0.0, -20.0, -40.0, -60.0, 20.0, 40.0, 60.0)
    camera_ahead_m: float = 0.20
    camera_above_m: float = 0.30
    target_range_m: float = 0.55
    target_above_m: float = 0.24
    # The second look, straight in front of the tag: the first of these (range, height above it) that solves.
    close_views_m: tuple = ((0.25, 0.03), (0.30, 0.03), (0.25, 0.08), (0.22, 0.0), (0.35, 0.06))
    # The tag reads the same at any image rotation, so the close look may turn the camera about its view to
    # save the wrist a swing on the way to the grasp (`plan_close_look`'s `toward`).
    look_rolls_deg: tuple = (0.0, 45.0, 90.0, 135.0, 180.0, -135.0, -90.0, -45.0)


def heading_vector(up_b, heading_deg: float) -> np.ndarray:
    """Horizontal unit vector `heading_deg` left of the base's forward axis, about gravity's up."""
    up = np.asarray(up_b, dtype=float) / np.linalg.norm(up_b)
    forward_h = np.array([1.0, 0.0, 0.0]) - up * up[0]
    forward_h /= np.linalg.norm(forward_h)
    left = np.cross(up, forward_h)
    a = math.radians(heading_deg)
    return math.cos(a) * forward_h + math.sin(a) * left


def plan_look(joints, links, mount, camera_pos_b, target_b, up_b, seeds=(), roll_deg: float = 0.0):
    """Arm angles putting the wrist camera at `camera_pos_b`, looking at `target_b` with the image upright, or
    turned `roll_deg` about the view from upright."""
    view = np.asarray(target_b, dtype=float) - np.asarray(camera_pos_b, dtype=float)
    a = math.radians(roll_deg)
    about_view = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    camera = transform(optical_rotation(view, up_b) @ about_view, camera_pos_b)
    link6 = camera @ invert(mount.pose)
    return _solve(joints, links, link6[:3, 3], link6[:3, :3], _seeds(target_b, seeds), np.zeros(3))


def plan_search(joints, links, mount, up_b, q_start, params: SearchParams | None = None) -> list:
    """(heading, q) for each search stop that solves, in the order they are visited."""
    p = params or SearchParams()
    up = np.asarray(up_b, dtype=float) / np.linalg.norm(up_b)
    stops, q_prev = [], q_start
    for heading in p.headings_deg:
        h = heading_vector(up, heading)
        camera = MOUNT_B + p.camera_ahead_m * h + p.camera_above_m * up
        target = MOUNT_B + p.target_range_m * h + p.target_above_m * up
        q = plan_look(joints, links, mount, camera, target, up, [q_prev])
        if q is not None:
            stops.append((heading, q))
            q_prev = q
    return stops


def plan_close_look(joints, links, mount, box_pose_b, up_b, q_start, params: SearchParams | None = None,
                    toward=None):
    """Arm angles for a look straight at the tag from the first of `close_views_m` that solves; None if none.

    Without `toward` the image is upright. With it -- the arm's next pose -- the camera takes whichever of
    `look_rolls_deg` makes the least joint travel from `q_start` through the look to `toward`: every joint moves
    at the same ceiling, so the furthest-moving one sets each move's time."""
    p = params or SearchParams()
    tag = np.asarray(box_pose_b) @ tag_pose_box()
    centre, out = tag[:3, 3], tag[:3, 2]
    up = np.asarray(up_b) / np.linalg.norm(up_b)
    rolls = (0.0,) if toward is None else p.look_rolls_deg
    q_start = np.asarray(q_start, dtype=float)
    for range_m, above_m in p.close_views_m:
        solved = [q for q in (plan_look(joints, links, mount, centre + range_m * out + above_m * up, centre, up,
                                        [q_start], roll_deg=roll) for roll in rolls) if q is not None]
        if solved:
            if toward is None:
                return solved[0]
            travel = lambda q: float(np.max(np.abs(q - q_start)) + np.max(np.abs(np.asarray(toward) - q)))
            return min(solved, key=travel)
    return None


__all__ = ["PressParams", "PressPlan", "Lever", "plan_press", "press_capacity", "SearchParams", "plan_search",
           "plan_look", "plan_close_look", "heading_vector"]

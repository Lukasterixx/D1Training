"""What RViz draws for the key turned by the arm (`node.py --task key`), on top of the door demo's scene
(`combiner_door/robot/viz.scene_markers`, its `extra`): the key at the angle the robot measures (the start before the
fingers are on it), the arc its push point follows with the unlock marked, and on the floor where the walk up stops
(a green ring where it is sent, `key_plan.WALK_UP_ACCEPT_M`) and the area the push works from (`arm_turn.PUSH_*`), as base positions for the
box as the fix has it. numpy only; the marker class comes from the node.
"""
from __future__ import annotations

import numpy as np

from demos.combiner_door.robot import plan, viz

from .. import arm_turn as at
from .. import geometry as kg

KEY_ORANGE = (0.95, 0.55, 0.10, 1.0)


def key_pose(box, theta_deg: float) -> np.ndarray:
    """The key's frame (columns along it, off the door, up its top side; origin on the shaft at the door's face) in the
    frame `box` is given in."""
    u, n, t = (np.asarray(v) for v in kg.key_rotation_box(theta_deg))
    return np.asarray(box) @ plan.pose(np.column_stack([u, n, t]), kg.LOCK)


def key_markers(out, box, theta_deg: float) -> None:
    """The key's bar and platform at `theta_deg`, the push point's arc and the unlock on it."""
    frame = key_pose(box, theta_deg)
    for name, part in (("bar", at.KEY_BAR_BOX), ("platform", at.KEY_PAD_BOX)):
        centre = [0.5 * (lo + hi) for lo, hi in part]
        size = [hi - lo for lo, hi in part]
        out.add("key", "CUBE", frame @ plan.pose(None, centre), size, KEY_ORANGE)
    arc = [plan.transform(box, [at.platform_point(d)])[0] for d in np.linspace(at.START_DEG, at.END_DEG, 31)]
    out.add("key_arc", "LINE_STRIP", None, (0.006, 0, 0), viz.YELLOW, points=arc)
    out.add("key_arc", "SPHERE", plan.pose(None, plan.transform(box, [at.platform_point(at.UNLOCK_DEG)])[0]),
            (0.015, 0.015, 0.015), viz.GREEN)


def stance_markers(out, box) -> None:
    """Where the base may stand: the area the push works from (yellow) for the box as the fix has it, base positions on the floor for
    the box as the fix has it, its yaw taken as the script frame's (the push wants the door square to within 4 deg)."""
    lock = plan.transform(box, [kg.LOCK])[0]
    floor = float(np.asarray(box)[2, 3])
    from .key_plan import PUSH_DOOR_AHEAD_M, PUSH_LOCK_LEFT_M

    for (ahead, left), colour in (((PUSH_DOOR_AHEAD_M, PUSH_LOCK_LEFT_M), viz.YELLOW),):
        corners = [(lock[0] - a, lock[1] - l, floor) for a, l in
                   ((ahead[0], left[0]), (ahead[1], left[0]), (ahead[1], left[1]), (ahead[0], left[1]), (ahead[0], left[0]))]
        out.add("key_stance", "LINE_STRIP", None, (0.008, 0, 0), colour, points=corners)


def target_marker(out, box) -> None:
    """Where the walk up is sent: the script frame's origin (the stance as the odometry will report it), a green ring
    of `key_plan.WALK_UP_ACCEPT_M`."""
    from .key_plan import WALK_UP_ACCEPT_M

    floor = float(np.asarray(box)[2, 3])
    ring = [(WALK_UP_ACCEPT_M * np.cos(a), WALK_UP_ACCEPT_M * np.sin(a), floor) for a in np.linspace(0, 2 * np.pi, 25)]
    out.add("key_stance", "LINE_STRIP", None, (0.008, 0, 0), viz.GREEN, points=ring)


def extra_for(attempt):
    """The node's `scene_markers(extra=...)` for a key attempt (or the look, before one)."""
    def draw(out, box):
        theta = attempt.key_deg_shown() if attempt is not None else at.START_DEG
        key_markers(out, box, theta)
        stance_markers(out, box)
        target_marker(out, box)
    return draw


def status_text(attempt) -> str:
    """The key's part of the status line: what the fingers measure of it and the robot's check."""
    if attempt is None:
        return ""
    check = attempt.turn_check()
    if check.get("measured_deg") is None:
        return ""
    text = f"key {check['measured_deg']:+.0f} deg (fingers), lag {check['lag_mm']:.0f} mm"
    if check.get("verified") is not None:
        text += " | UNLOCKED (verified)" if check["verified"] else " | NOT verified unlocked"
    return text

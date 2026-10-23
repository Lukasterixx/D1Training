"""What RViz draws for the key inserted by the arm (`node.py --task insert`), on top of the door demo's scene
(`combiner_door/robot/viz.scene_markers`, its `extra`): the keyway where the fix puts it (its axis, and the slot at the
insertion's 45 degrees), the key where the arm's encoders put it in the hand, the search's spiral so far, and on the
floor the window the walk up must end in and where it is sent. numpy only; the marker class comes from the node.
"""
from __future__ import annotations

import numpy as np

from demos.combiner_door.robot import plan, viz

from .. import arm_turn as at
from .. import geometry as kg
from . import insert_plan as ip
from .key_viz import KEY_ORANGE

KEYWAY_SLOT_M = (0.015, 0.0035)       # the wards' slot: across the key's u, its t


def keyway_markers(out, box) -> None:
    """The keyway's axis (60 mm out of the seated key's back face) and its slot, at the insertion's angle."""
    seated, n, u, t = ip._frames(box)
    out.add("keyway", "LINE_STRIP", None, (0.003, 0, 0), viz.GREEN,
            points=[seated - ip.TRAVEL_M * n, seated + ip.BACK_OUT_M * n])
    frame = plan.pose(np.column_stack([u, n, t]), seated)
    out.add("keyway", "CUBE", frame, (KEYWAY_SLOT_M[0], 0.002, KEYWAY_SLOT_M[1]), viz.GREEN)


def held_key_markers(out, attempt) -> None:
    """The key in the hand where the encoders put it (its bar), while the hand holds it."""
    if attempt is None or attempt._measured is None or attempt._base_s is None or attempt.check.get("released"):
        return
    if attempt.check.get("held") is False:
        return
    origin, axes = ip.key_in_hand(attempt.arm.joints, attempt._measured, attempt._base_s)
    door_face = origin - ip.SEATED_OFF_DOOR_M * axes[:, 1]
    frame = plan.pose(axes, door_face)
    bar = at.KEY_BAR_BOX
    out.add("key_held", "CUBE", frame @ plan.pose(None, [0.5 * (lo + hi) for lo, hi in bar]),
            [hi - lo for lo, hi in bar], KEY_ORANGE)


def search_markers(out, attempt, box) -> None:
    """The spiral the search has swept, at the depth it pressed at."""
    if attempt is None or not attempt.search_path or attempt.check.get("contact_off_mm") is None:
        return
    off = attempt.check["contact_off_mm"] / 1000.0 - ip.PRESS_M
    seated, n, u, t = ip._frames(box)
    points = [seated + (off - ip.TRAVEL_M) * n + a * u + b * t for a, b in attempt.search_path[::3]]
    if len(points) >= 2:
        out.add("key_search", "LINE_STRIP", None, (0.0005, 0, 0), viz.YELLOW, points=points)


def vision_markers(out, attempt) -> None:
    """Where the wrist camera found the bezel's centre (on the model's plane), once it has."""
    if attempt is None or attempt._vision is None:
        return
    centre, _ = attempt._vision
    out.add("keyhole_seen", "SPHERE", plan.pose(None, centre), (0.004, 0.004, 0.004), viz.YELLOW)


def stance_markers(out, box) -> None:
    """The window the walk up must end in (`insert_plan.ACCEPT_*`), base positions on the floor."""
    seated, n, _, _ = ip._frames(box)
    right = np.asarray(box)[:3, :3] @ np.asarray(kg.LEVER_LEVEL_BOX)
    floor = float(np.asarray(box)[2, 3])
    (a0, a1), (r0, r1) = ip.ACCEPT_SEATED_AHEAD_M, ip.ACCEPT_LOCK_RIGHT_M
    corners = []
    for a, r in ((a0, r0), (a1, r0), (a1, r1), (a0, r1), (a0, r0)):
        p = seated + a * n - r * right
        corners.append((float(p[0]), float(p[1]), floor))
    out.add("key_stance", "LINE_STRIP", None, (0.008, 0, 0), viz.YELLOW, points=corners)


def extra_for(attempt):
    """The node's `scene_markers(extra=...)` for an insertion attempt (or the look, before one)."""
    from .key_viz import target_marker

    def draw(out, box):
        keyway_markers(out, box)
        held_key_markers(out, attempt)
        search_markers(out, attempt, box)
        vision_markers(out, attempt)
        stance_markers(out, box)
        target_marker(out, box)
    return draw


def status_text(attempt) -> str:
    """The insertion's part of the status line."""
    if attempt is None:
        return ""
    c = attempt.insert_check()
    parts = []
    vision = c.get("vision")
    if vision and vision.get("offset_mm"):
        parts.append(f"keyhole seen {vision['offset_mm'][0]:+.1f}/{vision['offset_mm'][1]:+.1f} mm off the model")
    elif vision:
        parts.append(f"keyhole not seen ({vision['found']}/{vision['frames']} frames)")
    if c.get("lie_drop_m") is not None:
        parts.append(f"lying ({100 * c['lie_drop_m']:.0f} cm down)")
    if "key_mm" in c and c.get("held") and not c.get("released"):
        off, a, b = c["key_mm"]
        parts.append(f"key {off:.1f} mm out, {a:+.1f}/{b:+.1f} mm across")
    if c.get("contact_off_mm") is not None and not c.get("seated"):
        parts.append(f"touched at {c['contact_off_mm']:.1f} mm")
    if c.get("searched_r_mm"):
        parts.append(f"searched to {c['searched_r_mm']:.1f} mm")
    if c.get("seated"):
        parts.append("SEATED")
    return " | ".join(parts)


def keyhole_outlines(prediction, K, camera_script, bezel=None, wide_m: float = 0.0) -> tuple[list, str]:
    """What the wrist camera's frame is drawn with through the insertion's key phases (`viz.debug_jpeg`): the bezel where
    the box model puts it (yellow), the area `keyhole.find_bezel` searches its centre over (grey, +-`SEARCH_M`), with
    `wide_m` the circle the image's own search looks inside (dark grey) and what it found (magenta), and the latest
    search's circle (green accepted, red not); plus a line of text saying what the search made of it.
    `prediction`: `insert_plan.keyhole_prediction` (centre, out of the door, u, t); `camera_script`: the wrist camera's
    optical frame in the same script frame (the arm's encoders)."""
    from demos.combiner_key import bezel_image, keyhole

    if K is None or camera_script is None:
        return [], ""
    centre, normal, u, t = (np.asarray(v, dtype=float) for v in prediction)
    w2c = np.linalg.inv(np.asarray(camera_script, dtype=float))
    angles = np.linspace(0.0, 2.0 * np.pi, 73)
    ring = lambda c, r: c + r * (np.cos(angles)[:, None] * u + np.sin(angles)[:, None] * t)
    half = keyhole.SEARCH_M + keyhole.BEZEL_RADIUS_M
    square = np.array([centre + a * half * u + b * half * t for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    out = [(keyhole._project(K, w2c, square), (150, 150, 150), 1, "search"),
           (keyhole._project(K, w2c, ring(centre, keyhole.BEZEL_RADIUS_M)), (0, 210, 255), 2, "model")]
    text = "keyhole: model yellow, search grey"
    if wide_m > 0:
        out.append((keyhole._project(K, w2c, ring(centre, wide_m)), (90, 90, 90), 1, "image search"))
        text += ", image magenta"
    image = getattr(bezel, "image", None) if bezel is not None else None
    if image and image.get("ok"):
        e = bezel_image.Ellipse(True, *image["centre_px"], *image["axes_px"], image["angle_deg"])
        out.append((e.points(n=72), (255, 0, 255), 2, "image"))
    if bezel is not None:
        found = np.asarray(bezel.centre, dtype=float)
        colour = (60, 220, 60) if bezel.ok else (60, 60, 230)
        out.append((keyhole._project(K, w2c, ring(found, keyhole.BEZEL_RADIUS_M)), colour, 2,
                    "found" if bezel.ok else "rejected"))
        text += (f" | {'FOUND' if bezel.ok else 'not found'} {1000 * bezel.a:+.1f}/{1000 * bezel.b:+.1f} mm, rim "
                 f"{100 * bezel.support:.0f}%" + ("" if bezel.ok else f" ({bezel.why})"))
        if wide_m > 0:
            text += (", searched round the image's find" if getattr(bezel, "how", "model") == "image" else
                     f", image: {(image or {}).get('why', 'nothing')}, searched round the model's")
    return out, text


"""The key turned by the arm: the D1's fingers push the key's platform from 45 degrees up to 45 down, planned in numpy.

Lukas, 2026-10-21: "I am curious if the arm itself can turn the lever" -- the same turn the front-right foot makes
(`geometry.INSERT_DEG` to `TURN_END_DEG`, clockwise as the robot sees it, unlocking past `UNLOCK_DEG`), scripted, with the
key already in the lock, and "the real lever takes about 5N to rotate in the lock".

**How the hand pushes** (after `demos/combiner/press.py`, the isolator handle's push): jaws shut, the hand comes in
pointing at the door and tipped `HAND_PITCH_DEG` down onto the platform about the lever's direction, the two fingers
side by side along the lever, and the fingertips' lower edge on the platform `PUSH_REACH_M` out from the shaft. The turn
is that whole hand pose rotated rigidly about the shaft, so the fingers meet the platform the same way at every angle;
the commanded edge sits `PRESS_DEPTH_M` inside the platform, so the fingers keep pressing. Held level (pitch 0, as the
isolator push was), the hand does not solve at the key at all: the lock is 0.42 m up, ~0.11 m above the Go2's base,
and the wrist and fingers are 0.2 m long.

**Where the robot stands**: the door `ARM_DOOR_AHEAD_M` ahead of the base and the lock `LOCK_LEFT_OF_BASE_M` to its
left -- the leg turn's stance mirrored, which puts the platform's arc in front of the arm's mount.

**How hard it can push** (`capacity_scan`, `press.press_capacity`'s quasi-static ceiling: the arm's weight plus J^T F
within `workspace.EFFORT_LIMIT_NM`, base level): at that stance and 0.28-0.33 m base heights, at least 1.47 N·m on the
key over the whole turn (the base yaw, Joint1, binds at 45 degrees up, where the push runs sideways), median ~1.7; 1.34-
1.54 with the stance 2-4 cm off. Lukas's 5 N is 0.65 N·m if it was felt where the fingers push.

Frames: the box frame of `demos/combiner/geometry.py` (+X out of the door, +Y the robot's right as it faces the door,
+Z up) for the key; the Go2's base frame (x ahead, y left) for the scan. No simulator.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from demos.cup.pick_demo.grasp import FINGER_X_RANGE_M, FINGERTIP_Z_M, JAW_CENTRE_LINK6

from . import geometry as kg

# --- the push -----------------------------------------------------------------------------------

#: Along the key from the shaft, where the fingertips meet the platform (it spans 48-148 mm): far out, for torque.
PUSH_REACH_M = 0.13
#: The fingertips' edge this far off the door's face, on the platform's top (which spans 17-67 mm off it).
CONTACT_OFF_DOOR_M = 0.035
#: The hand tipped down from pointing at the door, about the lever's direction (0 level, 90 pointing down at the
#: platform). 40 solves at a 0.28 m base, not from 0.31 (`capacity_scan`); 60 solves at 0.28-0.33.
HAND_PITCH_DEG = 60.0
#: How far inside the platform the edge is commanded, so the fingers keep pressing (`press.PressParams.press_depth_m`).
#: Not at the stop: held there 4 mm in, the arm pushed the key into its stop as hard as Joint1 could (100% of its
#: limit, 11 N; the first run, 2026-10-21), so the hold rests the edge on the platform ("resting").
PRESS_DEPTH_M = 0.004
#: Above the platform before coming down onto it, and out from the door before moving over it.
CLEARANCE_M = 0.03
STANDOFF_M = 0.08
#: The turn: from the key's start to its stop, at this speed (the leg turn's layer runs 0.6-1.2 rad/s at the foot). The
#: arm's lever (`geometry.ARM_LEVER`): since 2026-10-24 v4 in the changed lock, in level and round 45 degrees to its stop
#: (v3 went from 45 up to 45 down); unlocked past `UNLOCK_DEG`.
START_DEG = kg.ARM_LEVER.insert_deg
END_DEG = kg.ARM_LEVER.end_deg
UNLOCK_DEG = kg.ARM_LEVER.unlock_deg
TURN_SPEED_DEG_S = 15.0
#: The fingertips' lower edge in Link6 (the fingers' outer face, `FINGER_X_RANGE_M`, at the tips): what touches.
TIP_EDGE_LINK6 = (FINGER_X_RANGE_M[1], 0.0, FINGERTIP_Z_M)
#: The contact the lock's plant sees: a sphere this big inside the edge, its lowest point on the edge when the hand is
#: pitched as planned.
CONTACT_RADIUS_M = 0.005

#: The robot's own check that the key turned: its fingertips' edge on the estimated key past the unlock at the hold,
#: *and* the hand never this far behind its command in the turn. The edge alone reads 45 degrees on a key that never
#: moved once the fingers have slid off the platform and follow the command in the air (the 14 and 17 N locks,
#: 2026-10-21: lags of 96-160 mm; every key that turned lagged at most 20 mm).
TRACK_LIMIT_MM = 30.0

# --- the stance ---------------------------------------------------------------------------------

#: The door's face this far ahead of the base: the leg turn's 0.435 m plus a centimetre, since 2 cm closer does not
#: solve at a 0.33 m base.
ARM_DOOR_AHEAD_M = 0.44
#: The lock this far to the base's left (the leg turn's stance has it 8 cm to the right).
LOCK_LEFT_OF_BASE_M = 0.08
#: Where the push works from: every stance with the door and the lock in these ranges of the base solves the whole turn
#: at 0.28-0.33 m with the push's ceiling at least 1.31 N·m (`arc_rows`, 2026-10-21); 2 cm closer, or the lock in line
#: with the base at 0.48 m, does not at 0.33, nor does the door 0.52 m off.
PUSH_DOOR_AHEAD_M = (0.42, 0.48)
PUSH_LOCK_LEFT_M = (0.02, 0.12)
#: Where the walk may stop: the first time the look's estimate puts the door and the lock in these ranges, facing the
#: door to `ACCEPT_YAW_DEG` -- a range, as the real door pull accepts one (`combiner_door/robot/plan.ACCEPT_DISTANCE_M`),
#: not a point. Walked onto the point to 12 mm, P2Dingo's gait (stepping at 0.15 m/s or not at all) hunted round it for
#: the whole 12 s walk. The near part of the push's ranges: stopped, the gait drifts the base 1.5-4 cm back from the
#: door and up to 6 cm sideways in the settle (8 of 8, the first window, 2026-10-21, which stopped at 0.48 m and pushed
#: from 0.48-0.52).
ACCEPT_DOOR_AHEAD_M = (0.42, 0.45)
ACCEPT_LOCK_LEFT_M = (0.04, 0.12)
ACCEPT_YAW_DEG = 3.0


def contact_link6(pitch_deg: float = HAND_PITCH_DEG, radius_m: float = CONTACT_RADIUS_M) -> np.ndarray:
    """The contact sphere's centre in Link6: `radius_m` up the platform's normal from the tip edge, which in Link6 is
    `(-cos pitch, 0, -sin pitch)` for the hand as `hand_rotation` holds it."""
    p = math.radians(pitch_deg)
    return np.asarray(TIP_EDGE_LINK6) + radius_m * np.array([-math.cos(p), 0.0, -math.sin(p)])


def hand_rotation(theta_deg: float, pitch_deg: float = HAND_PITCH_DEG) -> np.ndarray:
    """Link6's axes (columns x, y, z) in the box frame with the key at `theta_deg`: z the approach (into the door, tipped
    down onto the platform), x the fingers' outer face (into the platform when level), y the jaw axis, along the key."""
    n = np.asarray(kg.OUT_OF_DOOR_BOX, dtype=float)
    top = np.asarray(kg.lever_top_box(theta_deg))
    p = math.radians(pitch_deg)
    z = -n * math.cos(p) - top * math.sin(p)
    x = -top * math.cos(p) + n * math.sin(p)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def platform_point(theta_deg: float, reach_m: float = PUSH_REACH_M, off_door_m: float = CONTACT_OFF_DOOR_M) -> np.ndarray:
    """A point on the platform's top, box frame: `reach_m` along the key, `off_door_m` off the door's face."""
    u, n, t = (np.asarray(v) for v in kg.key_rotation_box(theta_deg))
    return np.asarray(kg.LOCK) + reach_m * u + off_door_m * n + leg_top_m() * t


#: The arm's lever's shape as the lock's plant takes it (`geometry.ARM_LEVER`, in `unifp_train.leg_cfg.KEY_*`'s frame;
#: v3's is `leg_cfg`'s own, `tests/test_arm_turn.py` checks they agree): the platform's top above the shaft's axis, and
#: the bar and the platform as (along the key, off the door, up its top) boxes. v4's platform is 10 mm further out.
KEY_TOP_M = kg.ARM_LEVER.top_m
KEY_BAR_BOX = kg.ARM_LEVER.bar_box
KEY_PAD_BOX = kg.ARM_LEVER.pad_box


def leg_top_m() -> float:
    """The platform's top above the shaft's axis."""
    return KEY_TOP_M


def tip_target(phase: str, theta_deg: float) -> np.ndarray:
    """Where the tip edge is commanded, box frame, for a push phase with the key at `theta_deg`."""
    t = np.asarray(kg.lever_top_box(theta_deg))
    n = np.asarray(kg.OUT_OF_DOOR_BOX)
    on = platform_point(theta_deg)
    if phase == "pressed":
        return on - PRESS_DEPTH_M * t
    if phase == "resting":
        return on
    if phase == "above":
        return on + CLEARANCE_M * t
    if phase == "out":
        return on + CLEARANCE_M * t + STANDOFF_M * n
    raise ValueError(phase)


def jaw_centre(tip_box: np.ndarray, rotation_box: np.ndarray) -> np.ndarray:
    """The jaw centre (the arm IK's point, `JAW_CENTRE_LINK6`) for the tip edge at `tip_box` and Link6 at `rotation_box`."""
    return np.asarray(tip_box) + rotation_box @ (np.asarray(JAW_CENTRE_LINK6) - np.asarray(TIP_EDGE_LINK6))


def turn_angle(progress: float) -> float:
    """The key's commanded angle at `progress` through the turn: eased in and out."""
    s = min(1.0, max(0.0, progress))
    s = s * s * (3 - 2 * s)
    return START_DEG + (END_DEG - START_DEG) * s


def turn_seconds() -> float:
    return abs(END_DEG - START_DEG) / TURN_SPEED_DEG_S * 1.5      # the ease's peak speed is 1.5x its mean


def lock_torque_nm(push_n: float, at_m: float = PUSH_REACH_M) -> float:
    """A lock felt as `push_n` at `at_m` from the shaft, as a torque."""
    return push_n * at_m


def in_window(door_ahead_m: float, lock_left_m: float, yaw_error_deg: float) -> bool:
    """Whether a stance, the lock `door_ahead_m` ahead of and `lock_left_m` left of the base, is close enough."""
    return (ACCEPT_DOOR_AHEAD_M[0] <= door_ahead_m <= ACCEPT_DOOR_AHEAD_M[1]
            and ACCEPT_LOCK_LEFT_M[0] <= lock_left_m <= ACCEPT_LOCK_LEFT_M[1] and abs(yaw_error_deg) <= ACCEPT_YAW_DEG)


def stance_base(lock_env: np.ndarray, normal_env: np.ndarray, right_env: np.ndarray) -> np.ndarray:
    """The base's (x, y) for the push, from the lock's point on the door, the door's normal and the robot's right."""
    return np.asarray(lock_env) + ARM_DOOR_AHEAD_M * np.asarray(normal_env) + LOCK_LEFT_OF_BASE_M * np.asarray(right_env)


# --- the quasi-static scan, in the base frame ---------------------------------------------------

@dataclass
class ArcRow:
    theta_deg: float
    solved: bool
    q: np.ndarray | None = None
    capacity_nm: float = math.nan
    limiting_joint: str | None = None
    hold_use: float = math.nan
    trunk_clear: bool = False


def base_frame_box(door_ahead_m: float, lock_left_m: float, base_height_m: float) -> np.ndarray:
    """4x4: the box frame in the base frame for a box squarely ahead (its door's normal toward the robot), the lock
    `lock_left_m` left of the base and the base `base_height_m` above the floor."""
    m = np.eye(4)
    m[:3, :3] = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])   # box +X toward the robot, +Y its right
    lock_b = np.array([door_ahead_m, lock_left_m, kg.LOCK[2] - base_height_m])
    m[:3, 3] = lock_b - m[:3, :3] @ np.asarray(kg.LOCK)
    return m


def arc_rows(door_ahead_m: float = ARM_DOOR_AHEAD_M, lock_left_m: float = LOCK_LEFT_OF_BASE_M,
             base_height_m: float = 0.31, pitch_deg: float = HAND_PITCH_DEG, step_deg: float = 5.0,
             wrist_payload: bool = False) -> list[ArcRow]:
    """IK of the pressed hand along the turn, warm-started angle to angle, and the push it could make at each."""
    from demos.combiner.press import _seeds
    from demos.cup.pick_demo.grasp import arm_clear, solve_link6
    from position_only.workspace import load_urdf

    joints, links = load_urdf()
    box = base_frame_box(door_ahead_m, lock_left_m, base_height_m)
    offset = np.asarray(TIP_EDGE_LINK6)
    rows, q_prev = [], None
    for deg in np.arange(START_DEG, END_DEG + 1e-6, step_deg):
        deg = float(deg)
        rot = box[:3, :3] @ hand_rotation(deg, pitch_deg)
        on = box[:3, :3] @ platform_point(deg) + box[:3, 3]
        result = solve_link6(joints, links, on, rot, _seeds(on, [] if q_prev is None else [q_prev]), offset)
        if result is None:
            rows.append(ArcRow(deg, False))
            continue
        q_prev = np.asarray(result.q, dtype=float)
        push = -(box[:3, :3] @ np.asarray(kg.lever_top_box(deg)))
        force, joint, hold = _capacity(joints, links, q_prev, on, push, wrist_payload)
        rows.append(ArcRow(deg, True, q_prev, force * PUSH_REACH_M, joint, hold,
                           bool(arm_clear(joints, q_prev.reshape(1, 6))[0])))
    return rows


def _capacity(joints, links, q, point, push, wrist_payload: bool):
    """`press.press_capacity` with the wrist payload's weight optionally added: (N, the joint that binds, the largest
    share of a joint's limit the arm's own weight takes)."""
    import d1_ik
    from position_only.workspace import (DISTAL_LINKS, EFFORT_LIMIT_NM, GRAVITY, forward, simulated_coms,
                                         simulated_masses)

    frames, axes, origins = forward(joints, q.reshape(1, 6))
    masses, coms = simulated_masses(links, wrist_payload), simulated_coms(links, wrist_payload)
    hold = np.zeros(6)
    for name, (rot, pos) in frames.items():
        if name == "base_link":
            continue
        com = pos[0] + rot[0] @ coms[name]
        weight = masses[name] * GRAVITY
        for j in range(6):
            if name in DISTAL_LINKS[j]:
                hold[j] -= axes[0, j] @ np.cross(com - origins[0, j], weight)
    jac = d1_ik.jacobian(axes, origins, np.asarray(point, dtype=float).reshape(1, 3))[0][:3]
    per_newton = jac.T @ (np.asarray(push) / np.linalg.norm(push))
    force, joint = math.inf, None
    for i, (a, b) in enumerate(zip(per_newton, hold)):
        if abs(a) < 1e-9:
            continue
        for limit in (EFFORT_LIMIT_NM[i], -EFFORT_LIMIT_NM[i]):
            f = (limit - b) / a
            if 0.0 < f < force:
                force, joint = f, f"Joint{i + 1}"
    return force, joint, float(np.max(np.abs(hold) / EFFORT_LIMIT_NM))


def capacity_scan(base_heights=(0.28, 0.31, 0.33), door_offsets=(-0.02, 0.0, 0.02, 0.04),
                  lock_offsets=(-0.02, 0.0, 0.02)) -> list[dict]:
    """The push's quasi-static ceiling over the turn at the stance and around it, with and without the wrist camera."""
    out = []
    for h in base_heights:
        for payload in (False, True):
            for dd in door_offsets:
                for dl in lock_offsets:
                    rows = arc_rows(ARM_DOOR_AHEAD_M + dd, LOCK_LEFT_OF_BASE_M + dl, h, wrist_payload=payload)
                    solved = [r for r in rows if r.solved]
                    entry = {"base_height_m": h, "wrist_payload": payload, "door_ahead_m": round(ARM_DOOR_AHEAD_M + dd, 3),
                             "lock_left_m": round(LOCK_LEFT_OF_BASE_M + dl, 3), "angles": len(rows), "solved": len(solved)}
                    if len(solved) == len(rows):
                        worst = min(solved, key=lambda r: r.capacity_nm)
                        entry.update({"min_nm": round(worst.capacity_nm, 3), "min_at_deg": worst.theta_deg,
                                      "binding": worst.limiting_joint,
                                      "median_nm": round(float(np.median([r.capacity_nm for r in solved])), 3),
                                      "hold_use_max": round(max(r.hold_use for r in solved), 3),
                                      "trunk_clear": all(r.trunk_clear for r in solved)})
                    out.append(entry)
    return out


def main() -> int:
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(description="The arm's push on the key over the turn: IK and quasi-static ceiling.")
    parser.add_argument("--out", help="write the scan here (json)")
    args = parser.parse_args()
    rows = arc_rows()
    print(f"stance: door {ARM_DOOR_AHEAD_M} m ahead, lock {LOCK_LEFT_OF_BASE_M} m left; push {PUSH_REACH_M} m out, "
          f"hand pitched {HAND_PITCH_DEG} deg; base 0.31 m")
    for r in rows:
        print(f"  {r.theta_deg:+6.1f} deg: " + (f"{r.capacity_nm:.2f} N·m ({r.limiting_joint}), arm's weight "
                                                f"{100 * r.hold_use:.0f}% of a limit, trunk clear {r.trunk_clear}"
                                                if r.solved else "no IK solution"))
    scan = capacity_scan()
    for e in scan:
        print({k: e[k] for k in e})
    if args.out:
        Path(args.out).write_text(json.dumps({"arc": [{"theta_deg": r.theta_deg, "solved": r.solved,
                                                       "capacity_nm": r.capacity_nm, "binding": r.limiting_joint,
                                                       "hold_use": r.hold_use, "trunk_clear": r.trunk_clear}
                                                      for r in rows], "scan": scan}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

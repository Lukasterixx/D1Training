"""Metre-scale geometry of the combiner key demo: the key, the slot it goes into, where the robot stands. No simulator.

**The key** is Lukas's lever key (`assets/combinerbox_lever_v3.stl`, 2026-10-21): a 10 mm round shaft with two wards
15 mm across, 10 mm long, on a bar 15 mm thick and 20 mm tall that runs from 15 mm behind the shaft to 160 mm out, and a
2 mm plate 35 mm wide beside the bar from 48 to 148 mm out, its top flush with the bar's: the flat platform, 100 x 50 mm,
the foot presses. In the STL's own frame the shaft points -x (into the lock), the plate sticks out +x (toward the
robot), the platform faces +z, and the lever runs along -y -- which, with the shaft in the lock and the platform up,
is the robot's *left*. The turn Lukas asked for (the front-right foot pushing it clockwise, as the robot sees it, from
45 degrees up to 45 degrees down) needs it pointing right, toward the middle of the box, and no rotation of the part
gives both that and the platform up: the simulation uses the STL **mirrored left to right** (`KEY_MIRRORED`), the same
part printed handed the other way.

The key's frame, used everywhere here (and by the leg task's plant, `unifp_train/leg_cfg.KEY_*`): origin on the
shaft's axis at the bar's back face; `u` along the lever toward the platform, `n` off the door (toward the robot),
`t` up the platform's side. A seated key's back face stands on the slot's face.

**The slot**, adjusted so the key slides in (Lukas: "the slot will need to be adjusted"): a printed lock adapter over the
box's cam lock (`ADAPTER`), 16 mm thick and 1 mm proud of the door, whose keyway is the key's profile with 1.5 mm of
clearance round the shaft and 2 mm round the wards' ends, behind a 45-degree lead-in 5 mm wide on the shaft's four
sides. It takes a key turned to the insertion angle, 45 degrees up -- the angle the leg's turn starts from. A shaft
modelled square (10 x 10 mm) in a square hole stands in for the round one in a round hole: across the hole the
clearances are the same, and the wards set the key's roll either way.

**Where the robot stands** (the box frame of `demos/combiner/geometry.py`: +X out of the door, +Y the robot's right as
it faces the door, the root on the floor under the box's middle; the box stands on the floor, as the real one does,
not on the UniFP demos' plinth). The look is from the box's middle, where the head camera has the AprilTag straight
ahead; the turn needs the lock 8 cm to the right of the base, which puts the base 1.6 cm left of the box's edge; the arm
reaches the lock from 10 cm further back than that (`insert_ik_grid`, 2026-10-21: the IK holds the key level and
rolled 45 degrees at 0.50-0.60 m to the adapter's face, not at the turn's 0.42-0.49).
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from demos.combiner.geometry import GEOMETRY

KEY_STL = "combinerbox_lever_v3.stl"
#: The STL drawn mirrored left to right (see the module docstring).
KEY_MIRRORED = True
#: In the STL's frame: the shaft's axis (y, z) and the bar's back face (x), metres.
STL_AXIS_YZ = (0.0, 0.010)
STL_BACK_FACE_X = 0.010
KEY_MASS_KG = 0.053

#: The key's colliders, boxes in its (u, n, t) frame: the shaft (square, see the module docstring) and the wards, into
#: the slot; and a thin plate on the bar's back face, which seats on the adapter's face. Nothing else of the key
#: collides: it is held by a spring grip, not by the fingers, and only these parts touch the adapter.
KEY_SHAFT_BOX = ((-0.005, 0.005), (-0.010, 0.0), (-0.005, 0.005))
KEY_WARD_BOX = ((-0.0075, 0.0075), (-0.010, 0.0), (-0.00175, 0.00175))
KEY_SEAT_BOX = ((-0.015, 0.045), (0.0, 0.0006), (-0.010, 0.010))
#: The angle it goes in at, and the leg's turn: 45 degrees up to 45 down, unlocking past 30 down (the leg task's).
INSERT_DEG = -45.0
TURN_END_DEG = 45.0
UNLOCK_DEG = 30.0


@dataclass(frozen=True)
class Adapter:
    """The printed lock adapter: a block over the cam lock, its face `thickness + gap` off the door."""
    half_size: float = 0.030
    thickness: float = 0.016
    gap: float = 0.001
    shaft_half: float = 0.0065       # 13 mm square hole for the 10 mm shaft
    ward_half_u: float = 0.0095      # 19 mm long ward slot for the 15 mm wards
    ward_half_t: float = 0.00325     # 6.5 mm wide for their 3.5 mm
    depth: float = 0.012             # to the hole's floor; the 10 mm shaft does not reach it
    chamfer: float = 0.005           # the lead-in round the shaft's hole, 45 degrees
    friction: float = 0.3

    @property
    def face_off_door(self) -> float:
        return self.thickness + self.gap


ADAPTER = Adapter()

#: The key in the hand, `Link6`'s frame, roll -1 of `decoupled_arm.hand_rotation` (the one that solves at the
#: insertion stance): the jaws close across the bar's top and bottom 25 mm out from the shaft, between it and the plate,
#: the approach along the shaft. Link6's x is -u, y is +t, z is the shaft's way in (-n); the key's origin is 25 mm along
#: +x and 1 mm past the fingertips (z 0.1256), so they stay off the adapter as the key seats.
JAW_CENTRE_Z_M = 0.1051
KEY_GRASP_U_M = 0.025
KEY_ORIGIN_LINK6 = (KEY_GRASP_U_M, 0.0, 0.1266)
#: The key's u, n and t axes in Link6's frame (rows).
KEY_AXES_LINK6 = ((-1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))
GRASP_ROLL = -1

# --- the box frame ------------------------------------------------------------------------------

#: The lock's centre on the door's face, box frame (`GEOMETRY.lock`): 64 mm in from the free edge, midway up.
LOCK = GEOMETRY.lock
#: The robot's right, the lever's direction when level (+Y in the box frame), and out of the door (+X).
LEVER_LEVEL_BOX = (0.0, 1.0, 0.0)
OUT_OF_DOOR_BOX = (1.0, 0.0, 0.0)


def lever_dir_box(theta_deg: float) -> tuple[float, float, float]:
    """The lever's direction at `theta` (0 level, positive down: clockwise as the robot sees it), box frame."""
    a = math.radians(theta_deg)
    return (0.0, math.cos(a), -math.sin(a))


def lever_top_box(theta_deg: float) -> tuple[float, float, float]:
    """The platform's normal at `theta`, box frame."""
    a = math.radians(theta_deg)
    return (0.0, math.sin(a), math.cos(a))


def key_origin_box(theta_deg: float = INSERT_DEG, seated_off_m: float = 0.0) -> tuple[float, float, float]:
    """The key's origin (the shaft's axis at the bar's back face), box frame: on the adapter's face when seated,
    `seated_off_m` further out along the axis before it is."""
    x = LOCK[0] + ADAPTER.face_off_door + seated_off_m
    return (x, LOCK[1], LOCK[2])


def key_rotation_box(theta_deg: float) -> tuple[tuple[float, float, float], ...]:
    """The key's (u, n, t) axes in the box frame, as rows."""
    return (lever_dir_box(theta_deg), OUT_OF_DOOR_BOX, lever_top_box(theta_deg))


# --- stances, in the robot's frame at the look (x ahead, y left) ----------------------------------

#: The look: on the box's middle, its door this far ahead (the combiner door demo's look is the door pull's stance
#: plus 0.18 m: the handle 0.66-0.70 m off; the tag below it).
LOOK_DOOR_AHEAD_M = 0.70
#: The insertion stance: the adapter's face 0.55 m ahead of the base, the lock 8 cm to its right.
INSERT_FACE_AHEAD_M = 0.55
#: The turn stance: the door's face 0.435 m ahead (the middle of the leg task's frozen set), the lock 8 cm right.
TURN_DOOR_AHEAD_M = 0.435
LOCK_RIGHT_OF_BASE_M = 0.08


def stance_from_look(door_ahead_m: float) -> tuple[float, float]:
    """(ahead, left) of the look's base, metres, for a stance with the door `door_ahead_m` ahead and the lock
    `LOCK_RIGHT_OF_BASE_M` to the right, the box placed squarely `LOOK_DOOR_AHEAD_M` ahead of the look."""
    door_x = LOCK[0]
    ahead = LOOK_DOOR_AHEAD_M - door_ahead_m
    # The lock is at box +Y = the robot's right: -LOCK[1] to the left of the box's middle, as the robot faces it.
    left = -LOCK[1] + LOCK_RIGHT_OF_BASE_M
    del door_x
    return ahead, left


INSERT_STANCE = stance_from_look(INSERT_FACE_AHEAD_M - ADAPTER.face_off_door)
TURN_STANCE = stance_from_look(TURN_DOOR_AHEAD_M)

# --- placements ---------------------------------------------------------------------------------

#: The box's spread from attempt to attempt, about the look's nominal: its distance, sideways, and yaw.
PLACEMENT_AHEAD_JITTER_M = 0.02
PLACEMENT_SIDE_JITTER_M = 0.02
PLACEMENT_YAW_JITTER_DEG = 5.0

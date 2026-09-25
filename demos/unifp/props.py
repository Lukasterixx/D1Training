"""The standing demos' furniture, and where the grasp points land in UniFP's goal sphere.

The scripted cup pick and combiner demos run with the dog **lying down** and the object on the
floor. UniFP's policy stands, and its goal sphere is centred 0.49 m above the ground under the
robot (`interface.EE_GOAL_CENTER_OFFSET`) with a 0.30-0.58 m radius, so a floor-height object is
outside the region it was trained to reach at all. Hence a table for the cup and a post for the
box: the object is lifted into the workspace rather than the robot lowered onto it.

Heights are chosen against the measured workspace rather than by eye. F-093 found the tracking
error roughly triples in the bottom tenth of the sphere -- goals low and in front, where the policy
under-reaches -- and that error falls with distance from the task's own keep-out box. Both grasp
points here therefore sit within a few centimetres of the sphere's equator (pitch near 0, where the
tip is level with the 0.49 m centre) at a radius near 0.47 m, which the orientation probe measured
at 0.5-1.5 cm.

Everything is placed relative to the robot's spawn, in its yaw frame: +x forward, +y left, z up
from the ground plane. Dimensions of the box itself come from `demos.combiner.geometry`, which the
scripted demo uses, so the two demos work the same object.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from demos.combiner.geometry import GEOMETRY

# --- the cup on its table ---------------------------------------------------------------------

#: Table top height, chosen from the measured hand pose rather than by eye.
#:
#: The policy has no orientation command, so the direction the hand points at a goal is not a
#: choice -- it is a property of the goal. Measured over 105 held goals
#: (`orientation_probe.py`), the approach comes in 40-47 degrees nose-down at goals near the
#: sphere's equator and levels out as the goal rises: -12 degrees at radius 0.55, pitch 20. A cup
#: is taken off a table from the side, and a hand pointing 40 degrees down closes on the wall at
#: an angle and rolls the cup out of the jaws on the lift -- measured, before this height was
#: moved. So the table is set where the policy's own approach is nearly level.
#:
#: 0.60 m is also an ordinary table. The grasp point is 7.5 cm up a 10 cm cup, so the jaw centre
#: lands at 0.675 m: radius 0.547, pitch 19.8 degrees, which the probe measured at 0.9 cm.
TABLE_TOP_M = 0.60
#: Depth (x), width (y) of the top. Spawned as a solid pedestal rather than a top on legs: legs
#: are four more colliders for the dog's feet to find and nothing in either demo touches them.
TABLE_SIZE_M = (0.36, 0.50)
#: Distance from the robot to the *near face* of the table. The Go2's front feet stand at about
#: x = 0.19 m and its body reaches x = 0.22, so this leaves 0.22 m of clearance in front of it.
TABLE_NEAR_FACE_M = 0.44

#: The cup, as `pick_demo.cup_asset.build_cup_usd` builds it. 55 mm across against the jaws' 77.2 mm
#: open gap (`grasp.OPEN_GAP_M`), so this is an *outside* grasp -- the jaws close across the cup --
#: not the wall grasp F-065 needed for a 90 mm cup.
CUP_DIAMETER_M = 0.055
CUP_HEIGHT_M = 0.10
CUP_MASS_KG = 0.12
#: Nominal cup centre, on the table top, in the robot's yaw frame.
CUP_NOMINAL_XY_M = (0.515, 0.0)
#: Height of the grasp up the cup from its base: below the rim, on the wall, where an outside grasp
#: has the most material either side of the jaws.
CUP_GRASP_UP_M = 0.075
#: Lift asked of the demo: up, and in toward the robot. Straight up from a 0.60 m table leaves
#: the goal sphere -- 0.15 m above this grasp point is a radius of 0.64 against the trained 0.58 --
#: and drawing the cup in as it rises is what a person does with it anyway.
CUP_LIFT_UP_M = 0.12
CUP_LIFT_IN_M = 0.08
#: Height above the table top that counts as picked up.
CUP_SUCCESS_LIFT_M = 0.05

# --- the combiner box on its post -------------------------------------------------------------

#: The lever's spindle sits `GEOMETRY.bottom + GEOMETRY.height / 2` = 0.30 m above the box's root,
#: so a 0.38 m post puts the grasp point at 0.68 m: radius 0.53, pitch 21 degrees. Same reasoning
#: as the table -- that is where the hand arrives level, and a lever is gripped across a
#: horizontal bar, which wants the jaws above and below it and so the approach flat.
POST_TOP_M = 0.38
POST_SIZE_M = (0.22, 0.32)
#: Distance from the robot to the box's root. The scripted demo samples 0.62-0.70 m
#: (`geometry.sample_placement`); the handle stands 0.151 m proud of the root toward the robot, so
#: 0.64 m puts the grasp at a 0.52 m radius and 0.68 m at 0.55 -- inside the trained 0.58.
BOX_DISTANCE_RANGE_M = (0.64, 0.68)
BOX_BEARING_RANGE_DEG = (-12.0, 12.0)
BOX_YAW_JITTER_DEG = (-8.0, 8.0)
#: How far out the lever the jaws close, from the spindle. The scripted grip uses 90 mm on a 105 mm
#: lever; this is `GEOMETRY.handle_grasp_offset` + 5 mm, inboard of that, because the tip of the
#: lever is where a grip that slips comes off.
LEVER_GRASP_OFFSET_M = 0.075
#: Lever angle the demo drives to. Past `GEOMETRY.handle_release_deg` (45), as the scripted pull's
#: commanded 52 is, so arriving a few degrees short still releases the latch.
LEVER_TURN_DEG = 52.0
#: Door angle asked for, and the angle that counts as opened. The scripted grip reached 37-59
#: degrees against springs up to 0.4 N·m (F-073), so 30 is a pass either controller could earn.
DOOR_OPEN_DEG = 50.0
DOOR_SUCCESS_DEG = 30.0


@dataclass(frozen=True)
class CupSite:
    """A cup placement and the point the demo is trying to put the jaw centre on."""
    cup_xy_m: tuple[float, float]
    table_centre_x_m: float

    @property
    def cup_base_m(self) -> tuple[float, float, float]:
        return (self.cup_xy_m[0], self.cup_xy_m[1], TABLE_TOP_M)

    @property
    def grasp_point_m(self) -> tuple[float, float, float]:
        """On the cup's axis, `CUP_GRASP_UP_M` up from its base: where the jaw centre must end up."""
        return (self.cup_xy_m[0], self.cup_xy_m[1], TABLE_TOP_M + CUP_GRASP_UP_M)

    @property
    def lift_point_m(self) -> tuple[float, float, float]:
        x, y, z = self.grasp_point_m
        approach = self.approach_unit
        return (x - approach[0] * CUP_LIFT_IN_M, y - approach[1] * CUP_LIFT_IN_M,
                z + CUP_LIFT_UP_M)

    @property
    def approach_unit(self) -> tuple[float, float, float]:
        """Direction the hand comes in along: horizontally outward from the robot to the cup.

        A cup on a table is approached from the side rather than from above. Straight down would
        have the jaws closing onto the rim, and a cup is a cylinder -- there is nothing to hold
        there. The jaws must straddle the wall.
        """
        x, y = self.cup_xy_m
        norm = math.hypot(x, y) or 1.0
        return (x / norm, y / norm, 0.0)

    @property
    def cup_axis_unit(self) -> tuple[float, float, float]:
        """The cup stands upright, so the axis the jaws close across is vertical."""
        return (0.0, 0.0, 1.0)


@dataclass(frozen=True)
class BoxSite:
    """A combiner-box placement on its post, and the lever point the jaws must close on."""
    distance_m: float
    bearing_deg: float
    yaw_deg: float

    @property
    def root_m(self) -> tuple[float, float, float]:
        angle = math.radians(self.bearing_deg)
        return (self.distance_m * math.cos(angle), self.distance_m * math.sin(angle), POST_TOP_M)

    @property
    def quaternion(self) -> tuple[float, float, float, float]:
        half = math.radians(self.yaw_deg) / 2
        return (math.cos(half), 0.0, 0.0, math.sin(half))

    def world(self, local: tuple[float, float, float]) -> tuple[float, float, float]:
        """A point in the enclosure's frame, in the robot's yaw frame."""
        angle = math.radians(self.yaw_deg)
        x, y, z = local
        root = self.root_m
        return (root[0] + math.cos(angle) * x - math.sin(angle) * y,
                root[1] + math.sin(angle) * x + math.cos(angle) * y,
                root[2] + z)

    def direction(self, local: tuple[float, float, float]) -> tuple[float, float, float]:
        """A direction in the enclosure's frame, in the robot's yaw frame (no translation)."""
        angle = math.radians(self.yaw_deg)
        x, y, z = local
        return (math.cos(angle) * x - math.sin(angle) * y,
                math.sin(angle) * x + math.cos(angle) * y, z)

    @staticmethod
    def _about_hinge(local, door_deg):
        """Carry a point in the enclosure frame round the door's hinge (its local -Z)."""
        if not door_deg:
            return local
        angle = math.radians(-door_deg)
        hx, hy = GEOMETRY.hinge[0], GEOMETRY.hinge[1]
        x, y = local[0] - hx, local[1] - hy
        return (hx + math.cos(angle) * x - math.sin(angle) * y,
                hy + math.sin(angle) * x + math.cos(angle) * y, local[2])

    def grasp_point_m(self, handle_deg: float = 0.0, door_deg: float = 0.0) -> tuple[float, float, float]:
        """Where the jaw centre must be to hold the lever at these door and lever angles.

        `GEOMETRY.handle_at_angle` is the same construction at the geometry's own
        `handle_grasp_offset`; this uses `LEVER_GRASP_OFFSET_M` instead. The lever extends from its
        spindle toward the enclosure's -y and a positive lever angle lowers it, so turning it moves
        the grasp point down *and* in, not down alone -- which is why the demo's turn phase follows
        an arc rather than a vertical line.
        """
        sx, sy, sz = GEOMETRY.spindle
        lever = math.radians(handle_deg)
        local = (sx + GEOMETRY.handle_projection,
                 sy - LEVER_GRASP_OFFSET_M * math.cos(lever),
                 sz - LEVER_GRASP_OFFSET_M * math.sin(lever))
        return self.world(self._about_hinge(local, door_deg))

    def lever_axis_unit(self, handle_deg: float = 0.0, door_deg: float = 0.0):
        """Direction the lever bar runs in: the axis the jaws must close *across*."""
        lever = math.radians(handle_deg)
        local = (0.0, -math.cos(lever), -math.sin(lever))
        if door_deg:
            angle = math.radians(-door_deg)
            local = (math.cos(angle) * local[0] - math.sin(angle) * local[1],
                     math.sin(angle) * local[0] + math.cos(angle) * local[1], local[2])
        return self.direction(local)

    def approach_unit(self, door_deg: float = 0.0) -> tuple[float, float, float]:
        """Direction the hand travels along to reach the lever: from the robot into the door.

        The door's outward normal is the enclosure's +x, carried round the hinge as the door
        opens; the approach is its negative.
        """
        local = (1.0, 0.0, 0.0)
        if door_deg:
            angle = math.radians(-door_deg)
            local = (math.cos(angle), math.sin(angle), 0.0)
        out = self.direction(local)
        return (-out[0], -out[1], -out[2])


def sample_cup_site(rng) -> CupSite:
    """A cup placement on the table. Only `uniform` is required of `rng`."""
    x = float(rng.uniform(CUP_NOMINAL_XY_M[0] - 0.03, CUP_NOMINAL_XY_M[0] + 0.03))
    y = float(rng.uniform(-0.08, 0.08))
    return CupSite(cup_xy_m=(x, y), table_centre_x_m=TABLE_NEAR_FACE_M + TABLE_SIZE_M[0] / 2)


def sample_box_site(rng) -> BoxSite:
    """A box placement on its post, facing the robot, as `geometry.sample_placement` does."""
    distance = float(rng.uniform(*BOX_DISTANCE_RANGE_M))
    bearing = float(rng.uniform(*BOX_BEARING_RANGE_DEG))
    yaw = bearing + 180.0 + float(rng.uniform(*BOX_YAW_JITTER_DEG))
    return BoxSite(distance_m=distance, bearing_deg=bearing, yaw_deg=yaw)


def side_stance(site: BoxSite, stance_deg: float) -> BoxSite:
    """`site` turned about its grasp point by `stance_deg`: the robot stands off the door's normal.

    The grasp point stays where it was in the robot's frame -- the same reach, the same place in the
    goal sphere -- and only the directions the lever and the door move in turn with the box. Positive
    puts the robot on the latch side (the lever's spindle, the enclosure's +y, the robot's right when it
    faces the door squarely): there turning the lever down also draws the handle toward the robot, and
    the opening door swings away from it. Negative puts it on the hinge side, the other way round.
    """
    if not stance_deg:
        return site
    grasp = site.grasp_point_m()
    turned = BoxSite(site.distance_m, site.bearing_deg, site.yaw_deg - stance_deg)
    moved = turned.grasp_point_m()
    root = site.root_m
    x, y = root[0] + grasp[0] - moved[0], root[1] + grasp[1] - moved[1]
    return BoxSite(math.hypot(x, y), math.degrees(math.atan2(y, x)), turned.yaw_deg)


def goal_sphere_coords(point_m) -> tuple[float, float, float]:
    """(radius, pitch, yaw) of a point in the robot's yaw frame, as UniFP commands goals.

    Pure arithmetic against `EE_GOAL_CENTER_OFFSET`, so a placement can be checked against the
    trained ranges before a simulator is started.
    """
    from unifp_isaaclab import interface

    cx, cy, cz = interface.EE_GOAL_CENTER_OFFSET
    dx, dy, dz = point_m[0] - cx, point_m[1] - cy, point_m[2] - cz
    radius = math.sqrt(dx * dx + dy * dy + dz * dz)
    return radius, math.asin(dz / radius) if radius else 0.0, math.atan2(dy, dx)


def within_trained_workspace(point_m) -> bool:
    """True where UniFP's goal generator would be willing to place a goal."""
    from unifp_isaaclab import interface

    radius, pitch, yaw = goal_sphere_coords(point_m)
    return (interface.EE_GOAL_RADIUS_RANGE[0] <= radius <= interface.EE_GOAL_RADIUS_RANGE[1]
            and interface.EE_GOAL_PITCH_RANGE[0] <= pitch <= interface.EE_GOAL_PITCH_RANGE[1]
            and interface.EE_GOAL_YAW_RANGE[0] <= yaw <= interface.EE_GOAL_YAW_RANGE[1])

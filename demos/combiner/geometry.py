"""Metre-scale geometry of the combiner box and seeded placement; no simulator dependencies.

The box is Lukas's real enclosure (Shoals PV array d.c isolator, photographed 2026-10-19): 830 mm tall,
630 mm wide and 300 mm deep, standing on the floor, its door hinged on the right edge and held by a
quarter-turn cam lock midway up the left edge. Measured by Lukas: the outside dimensions, the isolator
handle's centre 160 mm below the top, the tag's top edge 150 mm below that centre, and the lock's centre
64 mm in from the left edge. Not measured, and assumed here: the handle's and the tag's horizontal place
(the door's centreline; the photo puts both within a few centimetres of it), the lever's own shape (kept
from the earlier proxy, so the grasp records stay comparable), wall and door thickness, and every mass.
Local +X points out of the door, +Y is right when looking at it, +Z is up.
The root is on the floor directly below the centre of the enclosure.

The handle is the isolator's rotary handle. Its bar points from the spindle toward the hinge, as in the
photo, and turns downward about the door's normal. The lever's return spring is linear about a rest
angle below its stop, so it starts preloaded against the stop the way a real handle does, and is sized
by the torque it takes to hold the lever at `spring_reference_deg` -- the 45 degrees the latch needs
(Lukas, 2026-09-19). `--handle_torque_nm` sweeps it to find the most the arm can turn. Every spring
value is an assumption until the real handle is measured.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BoxGeometry:
    # 830 x 630 x 300 mm outside (measured). The body stands 5 mm off the floor so the door's lower edge
    # clears the ground plane as it swings, and its top stays at the measured 830 mm. `depth` is the body
    # behind the door: with the gap and the door the box is the measured 300 mm deep.
    width: float = 0.63
    height: float = 0.825
    depth: float = 0.284
    bottom: float = 0.005
    wall: float = 0.006
    door_thickness: float = 0.012
    door_gap: float = 0.004
    # +1: the hinge on the +Y edge (the right, looking at the door), as on the real box; the free edge and
    # the lock are then on -Y and the lever points +Y, toward the hinge. -1 mirrors the whole door.
    hinge_side: int = 1
    handle_below_top: float = 0.160        # isolator handle's centre (its spindle), measured
    handle_y: float = 0.0                  # assumed on the door's centreline
    lock_from_edge: float = 0.064          # cam lock's centre from the free edge, measured; midway up
    handle_projection: float = 0.065
    handle_length: float = 0.105
    handle_radius: float = 0.009
    handle_grasp_offset: float = 0.070
    handle_limit_deg: float = 60.0
    handle_release_deg: float = 45.0
    handle_reset_deg: float = 10.0
    handle_hold_deg: float = 50.0          # the H key's inspection hold, past the release
    spring_rest_deg: float = -15.0         # below the 0 degree stop: the preload
    spring_reference_deg: float = 45.0     # the spring is sized by its torque here
    # AprilTag tag36h11 id 0 on the door below the handle, printed from `dock_tag_id0_100mm_A4.pdf`: a
    # 100 mm black square cut out with its 12.5 mm white border, 125 mm square. `tag_size` is the black
    # square's edge (the size AprilTag and OpenCV pose estimation take). The cut sheet's top edge is 150 mm
    # below the handle's centre (measured); the tag is assumed centred under the handle.
    tag_id: int = 0
    tag_size: float = 0.100
    tag_top_below_handle: float = 0.150
    tag_standoff: float = 0.0006          # printed face above the door skin, clear of the decals
    latch_capture_deg: float = 0.5
    open_limit_deg: float = 110.0

    @property
    def door_x(self):
        return self.depth / 2 + self.door_gap + self.door_thickness / 2

    @property
    def top(self):
        return self.bottom + self.height

    @property
    def hinge(self):
        return (self.door_x, self.hinge_side * self.width / 2, self.bottom + self.height / 2)

    @property
    def spindle(self):
        return (self.door_x + self.door_thickness / 2, self.handle_y, self.top - self.handle_below_top)

    @property
    def lock(self):
        """Centre of the cam lock's face on the closed door: midway up the free edge."""
        return (self.door_x + self.door_thickness / 2,
                -self.hinge_side * (self.width / 2 - self.lock_from_edge), self.bottom + self.height / 2)

    @property
    def hinge_axis_z(self):
        """+1 if the door opens by positive rotation about +Z, -1 about -Z: outward either way."""
        return self.hinge_side

    @property
    def lever_axis_x(self):
        """+1 if the lever lowers by positive rotation about +X (out of the door), -1 about -X."""
        return -self.hinge_side

    def lever_direction(self, deg):
        """Unit vector from the spindle along the lever, enclosure frame, door closed."""
        a = math.radians(deg)
        return (0.0, self.hinge_side * math.cos(a), -math.sin(a))

    def lever_normal(self, deg):
        """The lever's upper side's outward normal: up when it is level, turning with it."""
        a = math.radians(deg)
        return (0.0, self.hinge_side * math.sin(a), math.cos(a))

    @property
    def handle(self):
        return self.handle_at_angle(0.0)

    @property
    def lever_axis_point(self):
        """Where the lever's centreline meets its rotation axis (the spindle's +X axis), door closed."""
        return (self.spindle[0] + self.handle_projection, self.spindle[1], self.spindle[2])

    @property
    def tag_outer_size(self):
        """Edge of the printed tag including its one-cell white border (tag36h11: 10 cells, 8 black)."""
        return self.tag_size * 10 / 8

    @property
    def tag_centre_y(self):
        return self.handle_y

    @property
    def tag_centre_z(self):
        return self.spindle[2] - self.tag_top_below_handle - self.tag_outer_size / 2

    @property
    def tag_centre(self):
        """Centre of the tag's face on the closed door, in the enclosure frame."""
        return (self.door_x + self.door_thickness / 2 + self.tag_standoff, self.tag_centre_y, self.tag_centre_z)

    def spring_stiffness(self, torque_nm):
        """N·m/rad of a return spring that needs `torque_nm` to hold the lever at the reference angle."""
        if not math.isfinite(torque_nm) or torque_nm <= 0:
            raise ValueError("handle torque must be finite and positive")
        return torque_nm / math.radians(self.spring_reference_deg - self.spring_rest_deg)

    def spring_torque(self, torque_nm, angle_deg):
        """The spring's torque (N·m) at `angle_deg` for a spring sized at `torque_nm`."""
        return self.spring_stiffness(torque_nm) * math.radians(angle_deg - self.spring_rest_deg)

    def spring_effort_limit(self, torque_nm):
        """A drive limit that never binds: 20% above the spring at the lever's stop, or held at H from 0."""
        stiffness = self.spring_stiffness(torque_nm)
        worst = max(self.handle_limit_deg - self.spring_rest_deg, self.handle_hold_deg)
        return 1.2 * stiffness * math.radians(worst)

    def handle_at_angle(self, angle_deg, handle_angle_deg=0.0):
        """Grasp point with independent door and lever rotation, in the enclosure frame.

        The door opens outward about `hinge_axis_z` (local +Z with the hinge on +Y, -Z on -Y).
        The lever extends from its spindle toward the hinge and a positive angle lowers it.
        """
        lever = math.radians(handle_angle_deg)
        grasp = (self.spindle[0] + self.handle_projection,
                 self.spindle[1] + self.hinge_side * self.handle_grasp_offset * math.cos(lever),
                 self.spindle[2] - self.handle_grasp_offset * math.sin(lever))
        angle = math.radians(self.hinge_axis_z * angle_deg)
        x, y = grasp[0] - self.hinge[0], grasp[1] - self.hinge[1]
        return (self.hinge[0] + math.cos(angle) * x - math.sin(angle) * y,
                self.hinge[1] + math.sin(angle) * x + math.cos(angle) * y, grasp[2])


GEOMETRY = BoxGeometry()
# Torque at 45 degrees (N·m). The first version's 1.2 N·m/rad spring needed 0.94 there; the CPU static
# model (`press.press_capacity`) allows the lying dog's arm 0.63-0.87 N·m at 45 degrees (Week 1 log,
# 2026-09-19), so that was beyond it. Lowered at Lukas's request so the arm can turn it; an assumption,
# not a measured handle.
DEFAULT_HANDLE_TORQUE_NM = 0.4
# The lever's own weight helps a downward push. From the collider volumes PhysX distributes its 0.12 kg
# over, its centre of mass sits 25 mm along the lever from the spindle: 0.029 N·m at horizontal.
LEVER_GRAVITY_NM = 0.029


@dataclass(frozen=True)
class Placement:
    position: tuple[float, float, float]
    yaw_deg: float

    @property
    def quaternion(self):
        half = math.radians(self.yaw_deg) / 2
        return (math.cos(half), 0.0, 0.0, math.sin(half))

    def world_point(self, point):
        angle = math.radians(self.yaw_deg)
        x, y, z = point
        return (self.position[0] + math.cos(angle) * x - math.sin(angle) * y,
                self.position[1] + math.sin(angle) * x + math.cos(angle) * y,
                self.position[2] + z)


# Default enclosure-centre distances from the dog's spawn, unchanged from the earlier proxy. With the real box and
# the dog standing (its base settles 5.7 cm back), on the CPU model at 30 seeded placements: the tag seen, the close
# look and the grip-and-pull plan at all 30 (the door to 20-40 deg in the plan), the push at 28. At 0.70-0.78 m the
# pull plans at 22 and the push at 12 (2026-10-19).
DEFAULT_DISTANCE_M = (0.62, 0.70)


def validate_region(distance=DEFAULT_DISTANCE_M, sweep_deg=45.0, yaw_jitter_deg=10.0):
    if not all(math.isfinite(v) for v in (*distance, sweep_deg, yaw_jitter_deg)):
        raise ValueError("Placement bounds must be finite")
    if not 0.60 <= distance[0] <= distance[1]:
        raise ValueError("box_range must be ordered and at least 0.60 m from the robot spawn")
    if not 0 <= sweep_deg <= 180:
        raise ValueError("sweep_deg must be between 0 and 180 (180 covers the full circle)")
    if not 0 <= yaw_jitter_deg <= 30:
        raise ValueError("yaw_jitter_deg must be between 0 and 30")


def sample_placement(rng, distance=DEFAULT_DISTANCE_M, sweep_deg=45.0, yaw_jitter_deg=10.0):
    """Uniform radius/bearing, facing the dog's spawn with small yaw variation.

    The default forward sector keeps the handle near the nominal arm workspace;
    it does not certify IK feasibility or clearance through a complete door swing.
    Accepts random.Random or a numpy Generator; only uniform() is required.
    """
    validate_region(distance, sweep_deg, yaw_jitter_deg)
    radius = float(rng.uniform(*distance))
    bearing = float(rng.uniform(-sweep_deg, sweep_deg))
    yaw = bearing + 180.0 + float(rng.uniform(-yaw_jitter_deg, yaw_jitter_deg))
    angle = math.radians(bearing)
    return Placement((radius * math.cos(angle), radius * math.sin(angle), 0.0), yaw)

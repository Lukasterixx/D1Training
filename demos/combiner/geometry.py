"""Metre-scale proxy geometry and seeded placement; no simulator dependencies.

Dimensions are demo assumptions, not measurements of the reference product.
Local +X points out of the door, +Y is right when looking at it, +Z is up.
The root is on the floor directly below the centre of the enclosure.

The lever's return spring is linear about a rest angle below its stop, so it starts preloaded
against the stop the way a real handle does, and is sized by the torque it takes to hold the lever
at `spring_reference_deg` -- the 45 degrees the latch needs (Lukas, 2026-09-19). The default is
lowered from the first version's 0.94 N·m there so the D1 can turn it; `--handle_torque_nm` sweeps it
to find the most the arm can turn. Every value is an assumption until the real handle is measured.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BoxGeometry:
    width: float = 0.36
    height: float = 0.40
    depth: float = 0.16
    bottom: float = 0.10
    wall: float = 0.006
    door_thickness: float = 0.012
    door_gap: float = 0.004
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
    # AprilTag tag36h11 on the door above the handle. `tag_size` is the black square's edge (the size
    # AprilTag and OpenCV pose estimation take); the white quiet zone adds a cell each side.
    tag_id: int = 0
    tag_size: float = 0.060
    tag_centre_y: float = 0.110
    tag_centre_z: float = 0.410
    tag_standoff: float = 0.0006          # printed face above the door skin, clear of the decals
    latch_capture_deg: float = 0.5
    open_limit_deg: float = 110.0

    @property
    def door_x(self):
        return self.depth / 2 + self.door_gap + self.door_thickness / 2

    @property
    def hinge(self):
        return (self.door_x, -self.width / 2, self.bottom + self.height / 2)

    @property
    def spindle(self):
        return (self.door_x + self.door_thickness / 2,
                self.width / 2 - 0.055, self.bottom + self.height / 2)

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

        The hinge axis is local -Z (right-hand rotation about +Z would open inward).
        The lever extends toward -Y; positive rotation about its +X spindle lowers it.
        """
        lever = math.radians(handle_angle_deg)
        grasp = (self.spindle[0] + self.handle_projection,
                 self.spindle[1] - self.handle_grasp_offset * math.cos(lever),
                 self.spindle[2] - self.handle_grasp_offset * math.sin(lever))
        angle = math.radians(-angle_deg)
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


def validate_region(distance=(0.62, 0.70), sweep_deg=45.0, yaw_jitter_deg=10.0):
    if not all(math.isfinite(v) for v in (*distance, sweep_deg, yaw_jitter_deg)):
        raise ValueError("Placement bounds must be finite")
    if not 0.60 <= distance[0] <= distance[1]:
        raise ValueError("box_range must be ordered and at least 0.60 m from the robot spawn")
    if not 0 <= sweep_deg <= 180:
        raise ValueError("sweep_deg must be between 0 and 180 (180 covers the full circle)")
    if not 0 <= yaw_jitter_deg <= 30:
        raise ValueError("yaw_jitter_deg must be between 0 and 30")


def sample_placement(rng, distance=(0.62, 0.70), sweep_deg=45.0, yaw_jitter_deg=10.0):
    """Uniform radius/bearing, facing the resting dog's spawn with small yaw variation.

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

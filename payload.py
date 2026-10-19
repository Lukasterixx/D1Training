"""What is bolted onto the Go2 + D1 beyond their URDFs: the lidar on the trunk, and the camera on the gripper.

* **Livox MID-360 and Lukas's mount for it**, on the front of the Go2's trunk. Both are cut out of Lukas's Go2 CAD
  (`description/mid360_from_cad.py`, 2026-10-19) into the Go2's `base_link` frame: the mount is a plate on the trunk and
  a cage over the lidar.
* **Intel RealSense D435i in Lukas's printed bracket**, on top of the pincer: the bracket's CAD (`demos/cup/pick_demo/
  bracket.py`, F-138), printed and fitted to the real arm (Lukas, 2026-10-19). Its pose is the saved mount,
  `demos/cup/pick_demo/assets/mounts/wrist_mount.json`.

Each part has a mass, a centre of mass and an inertia in the body it is fixed to, and box colliders. `weld.py` draws
the lidar and its mount on every robot, and for a training setup (`payload_physics`) adds the colliders and folds the
masses into the Go2's `base` and the D1's `Link6` (`combine`). `position_only.workspace.clear_of_body` keeps the arm
out of `CLEARANCE_BOXES_B`.

Masses and their sources. Two are published; two are estimates from the CAD's volume and want weighing:

| part | kg | source |
|---|---|---|
| MID-360 | 0.265 | published: Livox Mid-360 user manual, "approx. 265 g" |
| its mount | 0.155 | estimated: 124.9 cm^3 of the CAD's closed parts x 1.24 g/cm^3 (PLA, solid). A print with infill is lighter |
| D435i | 0.072 | published: Intel RealSense D400 series datasheet, D435/D435i "72 g" |
| wrist bracket | 0.027 | estimated: 21.6 cm^3 of the bracket's CAD x 1.24 g/cm^3 (PLA, solid) |

Numpy only. The wrist camera's pose is read from the saved mount when asked for (`wrist_parts`).
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
MESH_DIR = ROOT / "description" / "meshes" / "go2"
#: (name, file in `MESH_DIR`, display colour): drawn on the Go2's base, in its frame.
TRUNK_MESHES = (("mid360", "mid360.stl", (0.16, 0.16, 0.17)),
                ("mid360_mount", "mid360_mount.stl", (0.32, 0.33, 0.35)))

PLA_DENSITY_KG_M3 = 1240.0


@dataclass(frozen=True)
class Box:
    """A box collider: its centre and edge lengths (m), and its orientation (w, x, y, z), in the body's frame."""
    centre: tuple
    size: tuple
    quat_wxyz: tuple = (1.0, 0.0, 0.0, 0.0)

    @classmethod
    def from_bounds(cls, low, high) -> "Box":
        low, high = np.asarray(low, float), np.asarray(high, float)
        return cls(tuple((low + high) / 2), tuple(high - low))


@dataclass(frozen=True)
class Part:
    name: str
    body: str                 # the rigid body it is fixed to: the Go2's "base" or the D1's "Link6"
    mass_kg: float
    mass_source: str
    com: tuple                # in the body's frame, m
    inertia: tuple            # 3x3 about the centre of mass, body axes, kg m^2 (row-major, 9 numbers)
    colliders: tuple = ()


def box_inertia(mass: float, size, rotation=None) -> np.ndarray:
    """A solid box's inertia about its centre, in the body's axes when its own are `rotation` (3x3)."""
    a, b, c = size
    local = mass / 12.0 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])
    r = np.eye(3) if rotation is None else np.asarray(rotation, float)
    return r @ local @ r.T


def cylinder_inertia(mass: float, radius: float, height: float) -> np.ndarray:
    """A solid cylinder on the body's z axis, about its centre."""
    side = mass * (3 * radius * radius + height * height) / 12.0
    return np.diag([side, side, mass * radius * radius / 2.0])


def quat_to_matrix(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r) -> tuple:
    r = np.asarray(r, float)
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / n, x / n, y / n, z / n)


# --- on the trunk: from `description/mid360_from_cad.py`'s meshes, in `base_link` -----------------------------------

#: The mount's plate on the trunk, and the box round the lidar and the cage over it (the lidar is inside the second).
MOUNT_PLATE_BOX_B = ((0.0843, 0.2403), (-0.0406, 0.0400), (0.0552, 0.0942))
LIDAR_CAGE_BOX_B = ((0.1166, 0.2197), (-0.0327, 0.0323), (0.0882, 0.1662))
#: The lidar's housing: 65 mm round, 0.0958-0.1526 m up; its centre of mass at its convex hull's centroid.
MID360_COM_B = (0.1758, -0.0002, 0.1168)
#: The mount's closed parts' centroid (124.9 cm^3 of them).
MOUNT_COM_B = (0.1639, -0.0003, 0.0993)
MOUNT_VOLUME_M3 = 124.9e-6


def trunk_parts() -> tuple:
    plate = Box.from_bounds(*zip(*MOUNT_PLATE_BOX_B))
    cage = Box.from_bounds(*zip(*LIDAR_CAGE_BOX_B))
    mid360 = Part("mid360", "base", 0.265, "published: Livox Mid-360 user manual, approx. 265 g", MID360_COM_B,
                  tuple(cylinder_inertia(0.265, 0.0325, 0.057).flatten()), (cage,))
    mount_kg = round(MOUNT_VOLUME_M3 * PLA_DENSITY_KG_M3, 4)
    mount = Part("mid360_mount", "base", mount_kg,
                 "estimated: the CAD's closed parts, 124.9 cm^3 x 1.24 g/cm^3 (PLA, solid); weigh it", MOUNT_COM_B,
                 tuple(box_inertia(mount_kg, plate.size).flatten()), (plate,))
    return (mid360, mount)


#: Boxes, in the Go2's base frame, that the arm's links must keep clear of besides the trunk (`clear_of_body`).
CLEARANCE_BOXES_B = (MOUNT_PLATE_BOX_B, LIDAR_CAGE_BOX_B)


# --- on the gripper: the bracket's CAD and the saved camera mount, in `Link6` ----------------------------------------

#: The bracket's bounds and centroid in Link6, from `wrist_bracket_link6.stl` (three closed bodies, 21.57 cm^3).
BRACKET_BOX_L6 = ((-0.0648, -0.0324), (-0.0275, 0.0278), (0.0006, 0.0459))
BRACKET_COM_L6 = (-0.0468, 0.0, 0.024)
BRACKET_VOLUME_M3 = 21.57e-6
#: The D435's housing, 90 x 25 x 25 mm, and its centre in the colour optical frame (`camera_body.housing_size_m`,
#: `housing_centre_m`).
D435_SIZE_M = (0.0899, 0.025, 0.0251)
D435_CENTRE_OPTICAL_M = (0.0325, 0.0, -0.0082)


def wrist_parts(mount_pose=None) -> tuple:
    """The camera and its bracket on Link6. `mount_pose` is the colour optical frame in Link6 (4x4); by default the
    saved mount (`demos.cup.pick_demo.camera.resolve_mount`)."""
    if mount_pose is None:
        from demos.cup.pick_demo.camera import resolve_mount

        mount_pose = resolve_mount(None)[0].pose
    pose = np.asarray(mount_pose, float)
    rot = pose[:3, :3]
    centre = rot @ np.asarray(D435_CENTRE_OPTICAL_M) + pose[:3, 3]
    camera_box = Box(tuple(centre), D435_SIZE_M, matrix_to_quat(rot))
    camera = Part("d435i", "Link6", 0.072, "published: Intel RealSense D400 series datasheet, D435/D435i 72 g",
                  tuple(centre), tuple(box_inertia(0.072, D435_SIZE_M, rot).flatten()), (camera_box,))
    bracket_box = Box.from_bounds(*zip(*BRACKET_BOX_L6))
    bracket_kg = round(BRACKET_VOLUME_M3 * PLA_DENSITY_KG_M3, 4)
    bracket = Part("wrist_bracket", "Link6", bracket_kg,
                   "estimated: the CAD's 21.6 cm^3 x 1.24 g/cm^3 (PLA, solid); weigh it", BRACKET_COM_L6,
                   tuple(box_inertia(bracket_kg, bracket_box.size).flatten()), (bracket_box,))
    return (camera, bracket)


def all_parts(mount_pose=None) -> tuple:
    return trunk_parts() + wrist_parts(mount_pose)


# --- folding parts into a body ---------------------------------------------------------------------------------------

def combine(mass: float, com, inertia, parts) -> tuple:
    """A body's (mass, com, inertia about com) with `parts` fixed to it; all in the body's frame. Parallel-axis sums."""
    com = np.asarray(com, float)
    total = mass + sum(p.mass_kg for p in parts)
    new_com = (mass * com + sum(p.mass_kg * np.asarray(p.com, float) for p in parts)) / total

    def shifted(m, c, i):
        d = np.asarray(c, float) - new_com
        return np.asarray(i, float).reshape(3, 3) + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    tensor = shifted(mass, com, inertia) + sum(shifted(p.mass_kg, p.com, p.inertia) for p in parts)
    return total, new_com, tensor


def principal(tensor) -> tuple:
    """(diagonal inertia, principal axes as (w, x, y, z)) of a symmetric 3x3 tensor; a right-handed frame."""
    values, vectors = np.linalg.eigh(np.asarray(tensor, float))
    if np.linalg.det(vectors) < 0:
        vectors[:, 2] *= -1.0
    return tuple(values), matrix_to_quat(vectors)


def tensor_from_principal(diagonal, axes_wxyz) -> np.ndarray:
    r = quat_to_matrix(axes_wxyz)
    return r @ np.diag(diagonal) @ r.T

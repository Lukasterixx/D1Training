"""The AprilTag on the combiner door: its pattern, where it sits on the box, and finding it in a frame.

B's perception path is RealSense + AprilTags (docs/thesis_b_plan.md). The door carries one tag36h11
tag above the handle. Everything the arm needs about the box follows from that tag's pose and the
registered tag-to-box transform: in simulation the box's own geometry (`geometry.GEOMETRY`), on the
real box a measurement that has not been made yet.

Detection is OpenCV's ArUco module with its AprilTag 36h11 dictionary, then `solvePnP` with the IPPE
square solver. That is not the AprilTag C library `apriltag_ros` wraps on the robot: the two decode
the same family but refine corners differently, so a pose error measured here is this detector's.

Frames. The tag frame is OpenCV's: origin at the tag centre, x to the right and y up as the tag is
printed, z out of the tag towards whoever reads it. On the door that is enclosure +Y, +Z and +X.
Poses are 4x4 homogeneous matrices. numpy and OpenCV only; no Isaac.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .geometry import GEOMETRY

FAMILY = "tag36h11"
# tag36h11 id 0's 6x6 data cells, top row first as printed, 1 = white. Read from OpenCV's
# DICT_APRILTAG_36h11 (`cv2.aruco.generateImageMarker(dictionary, 0, 8)`); the tests decode it again.
TAG_BITS = {
    0: ((0, 0, 1, 0, 0, 0),
        (0, 1, 1, 0, 1, 0),
        (0, 0, 0, 1, 0, 1),
        (0, 0, 0, 1, 1, 0),
        (1, 0, 1, 1, 1, 0),
        (1, 0, 1, 0, 1, 1)),
}


def black_cells(tag_id: int = GEOMETRY.tag_id):
    """(row, column) of each black cell in the tag's 8x8 black square, row 0 at the top.

    The outer ring is the black border; the 6x6 inside is the code. The white quiet zone is one more
    cell all round, outside this grid.
    """
    bits = TAG_BITS[tag_id]
    for row in range(8):
        for col in range(8):
            border = row in (0, 7) or col in (0, 7)
            if border or not bits[row - 1][col - 1]:
                yield row, col


def tag_image(tag_id: int = GEOMETRY.tag_id, pixels_per_cell: int = 10) -> np.ndarray:
    """The printed tag, quiet zone included, as a greyscale image (for tests and for printing)."""
    image = np.full((10 * pixels_per_cell, 10 * pixels_per_cell), 255, np.uint8)
    for row, col in black_cells(tag_id):
        r, c = (row + 1) * pixels_per_cell, (col + 1) * pixels_per_cell
        image[r:r + pixels_per_cell, c:c + pixels_per_cell] = 0
    return image


def tag_pose_box(geometry=GEOMETRY) -> np.ndarray:
    """The tag frame in the enclosure frame, door closed: x = +Y, y = +Z, z = +X (out of the door)."""
    pose = np.eye(4)
    pose[:3, :3] = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    pose[:3, 3] = geometry.tag_centre
    return pose


def invert(pose) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = pose[:3, :3].T
    out[:3, 3] = -pose[:3, :3].T @ pose[:3, 3]
    return out


def corner_points(size: float) -> np.ndarray:
    """The black square's corners in the tag frame, in the order ArUco reports them and IPPE wants them:
    top-left, top-right, bottom-right, bottom-left."""
    h = size / 2
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


@dataclass
class TagDetection:
    tag_id: int
    corners_px: np.ndarray       # (4, 2) in `corner_points` order
    pose_cam: np.ndarray | None  # tag frame in the camera's optical frame; None without intrinsics
    reprojection_px: float       # RMS over the four corners (nan without a pose)
    side_px: float               # mean edge length in the image: how big the tag looked

    @property
    def range_m(self) -> float | None:
        return None if self.pose_cam is None else float(np.linalg.norm(self.pose_cam[:3, 3]))

    def as_dict(self) -> dict:
        return {"tag_id": self.tag_id, "range_m": None if self.range_m is None else round(self.range_m, 4),
                "side_px": round(self.side_px, 1),
                "reprojection_px": None if self.pose_cam is None else round(self.reprojection_px, 3),
                "corners_px": np.round(self.corners_px, 2).tolist()}


class TagDetector:
    """Finds tag36h11 tags in RGB frames and, given colour intrinsics, measures their poses.

    `tag_id` keeps only that tag (the turn sequence's use); None keeps every one (the console's). Without
    `intrinsic_matrix` it still finds and outlines tags but measures no pose. Pass the intrinsics the
    image was *rendered* with: the simulator draws square pixels about the image centre whatever the
    calibration says (F-070).
    """

    def __init__(self, intrinsic_matrix=None, tag_size: float = GEOMETRY.tag_size,
                 tag_id: int | None = GEOMETRY.tag_id):
        import cv2

        self.cv2 = cv2
        self.K = None if intrinsic_matrix is None else np.asarray(intrinsic_matrix, dtype=float).reshape(3, 3)
        self.tag_size = float(tag_size)
        self.tag_id = None if tag_id is None else int(tag_id)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)

    def detect(self, rgb) -> list[TagDetection]:
        cv2 = self.cv2
        image = np.asarray(rgb)
        grey = image if image.ndim == 2 else cv2.cvtColor(image[..., :3].astype(np.uint8), cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(grey)
        found = []
        if ids is None:
            return found
        points = corner_points(self.tag_size)
        for quad, tag_id in zip(corners, ids.flatten()):
            if self.tag_id is not None and int(tag_id) != self.tag_id:
                continue
            quad = quad.reshape(4, 2).astype(float)
            side = float(np.mean(np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)))
            if self.K is None:
                found.append(TagDetection(int(tag_id), quad, None, math.nan, side))
                continue
            ok, rvec, tvec = cv2.solvePnP(points, quad, self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            projected, _ = cv2.projectPoints(points, rvec, tvec, self.K, None)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - quad) ** 2, axis=1))))
            pose = np.eye(4)
            pose[:3, :3] = cv2.Rodrigues(rvec)[0]
            pose[:3, 3] = tvec.flatten()
            found.append(TagDetection(int(tag_id), quad, pose, error, side))
        return found


def box_pose(camera_pose_b, detection: TagDetection, geometry=GEOMETRY) -> np.ndarray:
    """The enclosure frame in the Go2 base frame, from one detection seen from `camera_pose_b`."""
    return np.asarray(camera_pose_b) @ detection.pose_cam @ invert(tag_pose_box(geometry))


def mean_pose(poses) -> np.ndarray:
    """Average of nearby poses: mean translation, and the rotation nearest the mean rotation matrix."""
    poses = np.asarray(poses, dtype=float)
    u, _, vt = np.linalg.svd(poses[:, :3, :3].mean(axis=0))
    rot = u @ vt
    if np.linalg.det(rot) < 0:
        u[:, -1] *= -1
        rot = u @ vt
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = poses[:, :3, 3].mean(axis=0)
    return out


def pose_error(estimate, truth) -> dict:
    """Translation error (mm) and rotation error (deg) between two poses in the same frame."""
    delta = invert(truth) @ estimate
    cos = np.clip((np.trace(delta[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    return {"position_mm": round(1000 * float(np.linalg.norm(estimate[:3, 3] - truth[:3, 3])), 2),
            "rotation_deg": round(math.degrees(math.acos(cos)), 3)}

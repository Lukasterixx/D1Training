"""From the head camera's AprilTag and the MID-360's points to the box placement the door script works from.

The UniFP door pull and its decoupled baseline are given the box's placement (`props.BoxSite`, the simulator's truth).
Here the script gets an estimate instead, made as P2Dingo's docking makes its fix (`dock_detector_node.cpp`): the tag
for the position, the lidar for the heading.
1. Each camera frame of the look, the tag found in the head camera's image (`demos.combiner.apriltag.TagDetector`,
   OpenCV's 36h11 decoder and IPPE pose), taken through the fixed mount and the base's pose since the attempt began
   (odometry: the simulator's, as the walk uses) into the script's frame, which is the stance, yaw only.
2. The tag's centre, averaged over the frames: the position. The tag's own heading, averaged: only a seed.
3. The lidar's frames over the same look, added up, and the door's face fitted as a line round the tag's centre
   (`mid360.face_line`, P2Dingo's wall fit), its window laid at the seed: the heading. Where no face is found, the
   tag's heading stands, and the record says so.
4. The box's root placed off the tag's centre at that heading, as a `BoxSite`. Its height and tilt are the known
   plinth's (`props.POST_TOP_M`, level). P2Dingo refits the tag's centre with the heading held; this does not.

Numpy only.
"""
from __future__ import annotations

import math

import numpy as np

from demos.combiner.apriltag import box_pose, mean_pose, pose_error
from demos.combiner.geometry import GEOMETRY
from demos.unifp import props

from . import head_camera, mid360


#: How far behind the door pull's stance the robot starts and looks from, metres. From the stance itself the head
#: camera cuts the tag off at a quarter of the placements; 0.10 m back it has it whole at all 200 tried (27 px to
#: spare), 0.15 m with more margin (the box 0.70-0.74 m off the stance). With the lever measured 27 mm off the door
#: (2026-10-20) the placements moved the box 29.5 mm closer so the handle keeps its distance (`props`), and the look
#: moves back as far, so the camera sees the box from where it did. Further back costs a longer walk in.
LOOK_BACK_M = 0.18


def yaw_of(rot) -> float:
    return math.atan2(float(rot[1, 0]), float(rot[0, 0]))


def base_in_spawn(base_pos, base_quat_wxyz, spawn_pos, spawn_yaw_quat_wxyz) -> np.ndarray:
    """The base's full pose (4x4) in the script's frame: the spawn position on the floor and the spawn heading.

    `base_pos` and `spawn_pos` are in the environment's frame (its origin taken off); `spawn_pos` has z 0, so a point's
    height in the script's frame is its height above the ground, as `props` measures heights."""
    spawn = head_camera.quat_to_matrix(spawn_yaw_quat_wxyz)
    pose = np.eye(4)
    pose[:3, :3] = spawn.T @ head_camera.quat_to_matrix(base_quat_wxyz)
    pose[:3, 3] = spawn.T @ (np.asarray(base_pos, float) - np.asarray(spawn_pos, float))
    return pose


def box_in_spawn(detection, base_spawn) -> np.ndarray:
    """The enclosure frame in the script's frame from one detection, through the head camera's fixed mount."""
    return np.asarray(base_spawn) @ box_pose(head_camera.optical_pose_base(), detection)


def site_from_pose(box_spawn) -> props.BoxSite:
    """The `BoxSite` whose root and yaw are the estimate's; the height is the plinth's (`props.POST_TOP_M`)."""
    x = float(box_spawn[0, 3]) - props.BOX_SCENE_SHIFT_M[0]
    y = float(box_spawn[1, 3]) - props.BOX_SCENE_SHIFT_M[1]
    return props.BoxSite(distance_m=math.hypot(x, y), bearing_deg=math.degrees(math.atan2(y, x)),
                         yaw_deg=math.degrees(yaw_of(box_spawn[:3, :3])))


def site_pose(site: props.BoxSite) -> np.ndarray:
    """A `BoxSite`'s enclosure frame in the script's frame, 4x4."""
    pose = np.eye(4)
    yaw = math.radians(site.yaw_deg)
    pose[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    pose[:3, 3] = site.root_m
    return pose


def estimate(poses) -> tuple[props.BoxSite, np.ndarray]:
    """The tag alone: the look's frames' enclosure poses (script frame) averaged, as a `BoxSite`, and the mean pose."""
    mean = mean_pose(poses)
    return site_from_pose(mean), mean


def tag_centre(box_spawn, geometry=GEOMETRY) -> np.ndarray:
    """The tag's centre (script frame) of an enclosure pose."""
    return (np.asarray(box_spawn) @ np.append(geometry.tag_centre, 1.0))[:3]


def pose_from_tag_and_heading(centre, yaw: float, geometry=GEOMETRY) -> np.ndarray:
    """The level enclosure frame whose tag sits at `centre` (script frame), turned to `yaw`."""
    pose = np.eye(4)
    pose[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    pose[:3, 3] = np.asarray(centre, float) - pose[:3, :3] @ np.asarray(geometry.tag_centre)
    return pose


def lidar_estimate(poses, cloud) -> tuple[props.BoxSite, np.ndarray, mid360.FaceFit]:
    """The tag's position with the lidar's heading (module docstring): the `BoxSite`, its pose and the face's fit.

    `poses` are the look's per-frame enclosure poses from the tag, `cloud` the look's lidar returns (n, 3), both in the
    script's frame. Without a face round the tag, the tag's own heading."""
    mean = mean_pose(poses)
    centre = tag_centre(mean)
    seed = yaw_of(mean[:3, :3])
    fit = mid360.face_line(cloud, centre, seed + math.pi)
    yaw = fit.heading - math.pi if fit.ok else seed
    pose = pose_from_tag_and_heading(centre, yaw)
    return site_from_pose(pose), pose, fit


def compare(estimate_pose, estimate_site: props.BoxSite, truth_site: props.BoxSite) -> dict:
    """How far the look was off, against the placement the simulator used: the full pose the tag gave, and the
    `BoxSite` the script runs on (which has the plinth's height and no tilt by construction)."""
    truth = site_pose(truth_site)
    full = pose_error(estimate_pose, truth)
    used = pose_error(site_pose(estimate_site), truth)
    delta = np.asarray(estimate_pose)[:3, 3] - truth[:3, 3]
    return {"pose_position_mm": full["position_mm"], "pose_rotation_deg": full["rotation_deg"],
            "pose_offset_mm_xyz": np.round(1000 * delta, 2).tolist(),
            "site_position_mm": used["position_mm"], "site_yaw_deg": used["rotation_deg"],
            "estimate_site": {"distance_m": round(estimate_site.distance_m, 4),
                              "bearing_deg": round(estimate_site.bearing_deg, 3),
                              "yaw_deg": round(estimate_site.yaw_deg, 3)}}

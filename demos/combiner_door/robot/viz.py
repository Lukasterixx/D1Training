"""What the robot program draws in RViz, and the message plumbing for it: the robot model's transforms from the URDF,
the box and the plan as markers, the lidar's face points, the camera's frame with the tag outlined.

The ROS message classes are passed in, so everything here but the JPEG (OpenCV) runs without ROS. Frames: the box,
the plan and the face points are in `combiner_door/script` (the door pull's stance, `plan.py`); the camera's field in
`base_link`; the robot model under `d1/` (measured) and `d1_plan/` (commanded: the ghost).
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import d1_hardware
import d1_ik
from demos.combiner.geometry import GEOMETRY
from demos.combiner_door import mid360, perception
from demos.combiner_door.robot import plan

SCRIPT_FRAME = "combiner_door/script"


# --- poses ------------------------------------------------------------------------------------------------------------

def quat_to_matrix(wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in wxyz)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(rot) -> tuple[float, float, float, float]:
    """(w, x, y, z) of a rotation matrix."""
    m = np.asarray(rot, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (m[k, j] - m[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (m[j, i] + m[i, j]) / s
    q[1 + k] = (m[k, i] + m[i, k]) / s
    return tuple(q)


def rpy_matrix(roll, pitch, yaw) -> np.ndarray:
    return plan.rpy_matrix(roll, pitch, yaw)


def axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


# --- the robot model ----------------------------------------------------------------------------------------------------

class UrdfModel:
    """A URDF's joints, enough to give every link its transform from its parent."""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.joints = []
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            axis = joint.find("axis")
            self.joints.append({
                "name": joint.get("name"), "type": joint.get("type"),
                "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"),
                "origin": plan.pose(rpy_matrix(*rpy), xyz),
                "axis": np.array([float(v) for v in (axis.get("xyz") if axis is not None else "1 0 0").split()])})

    def transforms(self, values: dict):
        """(parent, child, 4x4) for every joint: fixed ones always, moving ones where `values` has a number."""
        out = []
        for j in self.joints:
            if j["type"] == "fixed":
                out.append((j["parent"], j["child"], j["origin"]))
                continue
            value = values.get(j["name"])
            if value is None or not math.isfinite(value):
                continue
            if j["type"] in ("revolute", "continuous"):
                motion = plan.pose(axis_angle(j["axis"], value))
            else:
                motion = plan.pose(None, j["axis"] * value)
            out.append((j["parent"], j["child"], j["origin"] @ motion))
        return out


def description_for_pc(path, mesh_root: str) -> str:
    """The URDF's text with its placeholder mesh package (`package://d1_training/`) pointed at `mesh_root`."""
    return Path(path).read_text().replace("package://d1_training/", mesh_root)


def joint_values(leg_names, legs, arm_servo_deg, gripper_units) -> dict:
    """URDF joint values: the legs (radians, Unitree's order), the arm from its servo degrees (`d1_ik.from_servo_deg`)
    and the fingers from the gripper's units (`d1_hardware`'s unmeasured linear map). No arm: its joints absent."""
    values = {name: float(v) for name, v in zip(leg_names, legs)}
    if arm_servo_deg is not None:
        q = d1_ik.from_servo_deg(arm_servo_deg)[0]
        values.update({name: float(v) for name, v in zip(d1_ik.ARM_JOINTS, q)})
        travel = d1_hardware.gripper_units_to_finger_travel(40.8 if gripper_units is None else gripper_units)
        values["Joint7_1"], values["Joint7_2"] = travel, -travel
    return values


def transform_msg(cls, stamp, parent: str, child: str, p):
    msg = cls()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    p = np.asarray(p, dtype=float)
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (float(v) for v in p[:3, 3])
    w, x, y, z = matrix_to_quat(p[:3, :3])
    msg.transform.rotation.w, msg.transform.rotation.x = w, x
    msg.transform.rotation.y, msg.transform.rotation.z = y, z
    return msg


def chain_msgs(model: UrdfModel, values: dict, prefix: str, cls, stamp):
    return [transform_msg(cls, stamp, prefix + parent, prefix + child, p)
            for parent, child, p in model.transforms(values)]


# --- clouds -----------------------------------------------------------------------------------------------------------

def cloud_xyz(msg) -> np.ndarray:
    """x, y, z of a PointCloud2 (float32 fields), its no-return (0, 0, 0) points dropped."""
    offsets = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                      "offsets": [offsets["x"], offsets["y"], offsets["z"]], "itemsize": msg.point_step})
    array = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    points = np.stack([array["x"], array["y"], array["z"]], axis=-1).astype(float)
    keep = np.isfinite(points).all(axis=1) & (np.abs(points).sum(axis=1) > 1e-6)
    return points[keep]


def cloud_msg(cls, field_cls, stamp, frame: str, points, highlight):
    """x, y, z and an rgb (packed float) cloud: `highlight` points green, the rest grey."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    colour = np.where(np.asarray(highlight, dtype=bool), 0x33DD55, 0x999999).astype(np.uint32)
    data = np.zeros(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    data["x"], data["y"], data["z"], data["rgb"] = points[:, 0], points[:, 1], points[:, 2], colour
    msg = cls()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.height, msg.width = 1, len(points)
    msg.fields = [field_cls(name=n, offset=4 * i, datatype=field_cls.FLOAT32 if n != "rgb" else field_cls.UINT32,
                            count=1) for i, n in enumerate(("x", "y", "z", "rgb"))]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 16, 16 * len(points)
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


# --- the camera's frame -------------------------------------------------------------------------------------------------

def debug_jpeg(rgb, detections, text: str, scale: float = 0.5, outlines=()) -> bytes:
    """The colour frame at `scale`, the tag outlined with its range, `text` along the top, as JPEG bytes. `outlines`:
    (pixels (N, 2) at full size, BGR, thickness, label or "") drawn as closed polylines, NaN points dropped (the
    insertion's keyhole: `insert_viz.keyhole_outlines`)."""
    import cv2

    image = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR) if np.ndim(rgb) == 3 else \
        cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_GRAY2BGR)
    image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    for d in detections:
        quad = (np.asarray(d.corners_px) * scale).astype(np.int32)
        cv2.polylines(image, [quad], True, (60, 220, 60), 2)
        label = f"id {d.tag_id}" + (f"  {d.range_m:.2f} m" if math.isfinite(getattr(d, "range_m", math.nan)) else "")
        cv2.putText(image, label, tuple(int(v) for v in quad[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 220, 60), 1)
    for points, bgr, thickness, label in outlines:
        pts = np.asarray(points, dtype=float) * scale
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) < 2:
            continue
        cv2.polylines(image, [pts.astype(np.int32)], True, bgr, thickness, cv2.LINE_AA)
        if label:
            top = pts[np.argmin(pts[:, 1])]
            cv2.putText(image, label, (int(top[0]) + 4, int(top[1]) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, bgr, 1)
    cv2.rectangle(image, (0, 0), (image.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(image, text[:110], (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return jpeg.tobytes() if ok else b""


# --- the scene ----------------------------------------------------------------------------------------------------------

GREY, DOOR, BLACK, WHITE = (0.75, 0.76, 0.78, 0.35), (0.80, 0.81, 0.83, 0.85), (0.05, 0.05, 0.06, 1.0), (1, 1, 1, 1.0)
RED, GREEN, CYAN, YELLOW = (0.95, 0.25, 0.2, 1.0), (0.2, 0.85, 0.35, 1.0), (0.1, 0.8, 0.95, 1.0), (1.0, 0.85, 0.1, 1.0)


class _Markers:
    def __init__(self, cls, stamp):
        self.cls, self.stamp, self.items = cls, stamp, []

    def add(self, ns, kind, p=None, scale=(0.01, 0.01, 0.01), colour=WHITE, points=None, text=None,
            frame=SCRIPT_FRAME):
        m = self.cls()
        m.header.stamp = self.stamp
        m.header.frame_id = frame
        m.ns, m.id = ns, len([i for i in self.items if i.ns == ns])
        m.type = getattr(self.cls, kind)
        m.action = self.cls.ADD
        # Republished at 2 Hz: a marker the scene no longer has (an old hand target) goes rather than lingering.
        m.lifetime.sec, m.lifetime.nanosec = 1, 500_000_000
        p = np.eye(4) if p is None else np.asarray(p, dtype=float)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p[:3, 3])
        w, x, y, z = matrix_to_quat(p[:3, :3])
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = w, x, y, z
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in colour)
        if points is not None:
            from geometry_msgs.msg import Point
            m.points = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in points]
        if text is not None:
            m.text = text
        self.items.append(m)


def door_pose(box, angle_deg: float) -> np.ndarray:
    """The door's frame (the enclosure's, turned about the hinge by `angle_deg`), as the box frame `box` puts it."""
    return np.asarray(box) @ plan.about_hinge(angle_deg)


def box_markers(out: _Markers, box, door_deg: float, colour=DOOR) -> None:
    """The enclosure, its door at `door_deg`, the measured slot lever and the hub it turns on, the tag."""
    g = GEOMETRY
    z = g.bottom + g.height / 2
    out.add("box", "CUBE", np.asarray(box) @ plan.pose(None, (0.0, 0.0, z)), (g.depth, g.width, g.height), GREY)
    door = door_pose(box, door_deg)
    out.add("door", "CUBE", door @ plan.pose(None, (g.door_x, 0.0, z)), (g.door_thickness, g.width, g.height), colour)
    sx, sy, sz = g.spindle
    along_x = plan.pose(np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]))   # a cylinder's z onto x
    out.add("handle", "CYLINDER", door @ plan.pose(None, (sx + g.handle_gap / 2, sy, sz)) @ along_x,
            (g.handle_hub_diameter, g.handle_hub_diameter, g.handle_gap), BLACK)
    lever_x, side = sx + g.handle_projection, g.hinge_side
    out.add("handle", "CUBE", door @ plan.pose(None, (lever_x, sy + side * g.handle_span / 2, sz)),
            (g.handle_thickness, g.handle_span, g.handle_height), BLACK)
    for along in (0.0, g.handle_span):
        out.add("handle", "CYLINDER", door @ plan.pose(None, (lever_x, sy + side * along, sz)) @ along_x,
                (g.handle_height, g.handle_height, g.handle_thickness), BLACK)
    tx, ty, tz = g.tag_centre
    out.add("tag", "CUBE", door @ plan.pose(None, (tx, ty, tz)), (0.001, g.tag_outer_size, g.tag_outer_size), WHITE)
    out.add("tag", "CUBE", door @ plan.pose(None, (tx + 0.0008, ty, tz)), (0.001, g.tag_size, g.tag_size), BLACK)


def wrap_text(text: str, width: int = 36) -> list[str]:
    """`text` in lines of at most about `width` characters, broken at spaces, `|` and `;` starting new lines."""
    lines = []
    for part in text.replace(" | ", "\n").replace("; ", "\n").split("\n"):
        line = ""
        for word in part.split():
            if line and len(line) + 1 + len(word) > width:
                lines.append(line)
                line = word
            else:
                line = f"{line} {word}" if line else word
        if line:
            lines.append(line)
    return lines


def frustum_points(K, size, depth: float = 1.2, camera_pose=None):
    """LINE_LIST points of the camera's field, `base_link`, to `depth` metres, from `camera_pose` (the head's mount
    without one)."""
    if K is None or size is None:
        return None
    camera_pose = plan.CAMERA_POSE_BASE if camera_pose is None else np.asarray(camera_pose)
    w, h = size
    corners = [((u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1]) for u, v in ((0, 0), (w, 0), (w, h), (0, h))]
    far = [plan.transform(camera_pose, [(x * depth, y * depth, depth)])[0] for x, y in corners]
    apex = camera_pose[:3, 3]
    points = []
    for i in range(4):
        points += [apex, far[i], far[i], far[(i + 1) % 4]]
    return points


def scene_markers(cls, stamp, fix: plan.Fix, attempt, status: str, note: str, door: mid360.DoorReading, K=None,
                  size=None, plan_door: bool = False, door_task: bool = True, extra=None, camera_pose=None) -> list:
    """The scene. The door is drawn at the lidar's angle when it has one, else the script's; `plan_door` (a dry run,
    whose real door stays shut while the plan opens it): the script's. `door_task` False: no door pull's base path or
    grasp. `extra(out, box)`: another task's markers (`demos/combiner_key/robot/key_viz.py`)."""
    out = _Markers(cls, stamp)
    # The box with its door shut, which the door's angle is drawn from (the door close's look is of the open door).
    box = perception.site_pose(attempt.box_site() if attempt is not None else fix.site)
    commanded = attempt.commanded_door_deg() if attempt is not None else 0.0
    shown = door.angle_deg if door.ok and not plan_door else commanded
    box_markers(out, box, shown)
    ok, _ = fix.in_range()
    # The stance and the dog's outline on it; the base's path round the hinge through the pull.
    out.add("stance", "ARROW", None, (0.30, 0.03, 0.03), GREEN if ok else RED)
    outline = [(-0.32, -0.16, 0.0), (0.38, -0.16, 0.0), (0.38, 0.16, 0.0), (-0.32, 0.16, 0.0), (-0.32, -0.16, 0.0)]
    out.add("stance", "LINE_STRIP", None, (0.01, 0, 0), GREEN if ok else RED, points=outline)
    if door_task:
        door_deg = attempt.timing.door_deg if attempt is not None else 90.0
        path = [(*fix.site.base_pose_m(a)[:2], 0.0) for a in np.linspace(0.0, door_deg, 19)]
        out.add("base_path", "LINE_STRIP", None, (0.012, 0, 0), YELLOW, points=path)
        grasp = (attempt.grasp_site(fix.site) if attempt is not None else fix.site).grasp_point_m()
        out.add("grasp", "SPHERE", plan.pose(None, grasp), (0.02, 0.02, 0.02), YELLOW)
    if extra is not None:
        extra(out, box)
    last = getattr(attempt, "last_command", None) if attempt is not None else None
    if last is not None and last.hand_target_script is not None:
        hand = np.asarray(last.hand_target_script)
        out.add("hand_target", "SPHERE", plan.pose(None, hand), (0.03, 0.03, 0.03), CYAN)
        if last.approach_script is not None:
            out.add("hand_target", "ARROW", None, (0.008, 0.016, 0.02), CYAN,
                    points=[hand - 0.10 * np.asarray(last.approach_script), hand])
    segment = plan.face_segment(fix)
    if segment is not None:
        out.add("lidar_face", "LINE_STRIP", None, (0.012, 0, 0), CYAN, points=segment)
    frustum = frustum_points(K, size, camera_pose=camera_pose)
    if frustum is not None:
        out.add("camera", "LINE_LIST", None, (0.004, 0, 0), (0.6, 0.6, 1.0, 0.6), points=frustum, frame="base_link")
    words = [status, note]
    if door.ok:
        words.append(f"door: lidar {door.angle_deg:.1f} deg ({door.points} pts), script {commanded:.1f} deg")
    # Underscores, not spaces: RViz on Humble draws a space about four letters wide at any size, and every Unicode
    # space too (P2Dingo measured it, scripts/python/p2dingo-dock-viz.py).
    lines = [line.replace(" ", "_") for w in words if w for line in wrap_text(w)]
    out.add("text", "TEXT_VIEW_FACING", box @ plan.pose(None, (0.0, 0.0, GEOMETRY.top + 0.08 + 0.04 * len(lines))),
            (0, 0, 0.035), WHITE, text="\n".join(lines))
    return out.items

#!/usr/bin/env python3
"""The combiner door demo on the real Go2 + D1: a ROS 2 node on the dog. Start it with `run_on_dog.sh`.

    run_on_dog.sh                          # watch: find the box and say where it is; nothing moves
    run_on_dog.sh --stage pull             # the whole attempt as a DRY RUN: the plan drawn in RViz, nothing moves
    run_on_dog.sh --stage unfold --live    # really unfold the arm, then fold it again (Enter at each step)
    run_on_dog.sh --stage grip --live      # ... look, unfold, walk in, reach, grip, open, back off, fold
    run_on_dog.sh --task key --stage turn  # the key turned by the arm (demos/combiner_key/robot/key_plan.py), dry
    run_on_dog.sh --task key --stage turn --then_door pull  # the key turned, then the door pulled open from there
    run_on_dog.sh --task insert --stage insert  # the key handed to the arm and inserted into the lock (insert_plan.py)

What it does is `plan.py`'s (the simulated demo's sequence on the real robot's terms); this file is the robot's I/O:

- in: the look's camera -- the Go2's front camera by default (`/go2_front_camera/image_full/compressed`, P2Dingo's
  `p2dingo-front-camera`; `--camera front`, since 2026-10-22), or the RealSense with `--camera wrist|head` -- the
  RealSense on the wrist for the insertion's keyhole view (`/realsense/color/image_raw`, `camera_info`), the MID-360
  (`/livox/lidar`), the odometry
  (sport mode's `/sportmodestate`, or GLIM's `odom -> base_link` with `--odom tf`), the legs (`/lf/lowstate`, for
  drawing them), the website's joystick switch (`/web_teleop_enabled`), and the arm through `arm_link.py`'s socket;
- out, only with `--live`: the base on `/api/sport/request` (Move, StopMove), the arm through `arm_link.py`;
- out, always: what RViz on the operator's PC draws (`view_on_pc.sh`), under `/combiner_door/` (all of it in P2Dingo's
  zenoh allow-lists), and the record in `logs/combiner_door_robot/<run>/`.

A dry run commands nothing: its arm is a model that goes where it is sent and its base one that walks the command,
both drawn as the ghost robot, while the camera and lidar are the real ones. A live run looks, prints its plan and
waits for one Enter, then runs through and exits (`--step`: an Enter before each step that moves something,
`plan.CONFIRM_BEFORE`). Typing `a` + Enter stops it at once: the base stands, the arm holds where it is; then `r` +
Enter recovers (opens the jaws and backs the hand out if it may be at the lever, then folds), and `q` + Enter or
Ctrl-C quits with the arm where it is. A live run whose terminal closes stops and quits. The arm link holds the arm if
this node stops talking to it; this node sends StopMove on its way out.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import queue
import signal
import socket
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import d1_hardware  # noqa: E402
import d1_ik  # noqa: E402
from demos.combiner.apriltag import TagDetector  # noqa: E402
from demos.combiner_door import front_camera  # noqa: E402
from demos.combiner.geometry import GEOMETRY  # noqa: E402
from demos.combiner_door import mid360, perception  # noqa: E402
from demos.combiner_door.robot import arm_link, plan, viz  # noqa: E402
from demos.combiner_key.robot import insert_plan, insert_viz, key_plan, key_viz  # noqa: E402

ARM_LINK_PORT = 8095
LOG_ROOT = ROOT / "logs/combiner_door_robot"
#: The leg joints in Unitree's `LowState.motor_state` order.
LEG_JOINTS = [f"{leg}_{part}_joint" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")]
SPORT_API = {"balance_stand": 1002, "stop_move": 1003, "stand_up": 1004, "stand_down": 1005, "move": 1008,
             "classic_walk": 2049}
#: The insertion's postures (`plan.Command.posture`) as sport mode's requests: lying is StandDown, P2Dingo's "sit".
POSTURE_API = {"lie": "stand_down", "stand": "stand_up", "balance": "balance_stand"}
#: Each task's stages and the module that plans it.
TASK_STAGES = {"door": plan.STAGES, "key": key_plan.STAGES, "insert": insert_plan.STAGES + ("keyhole",)}
#: How long the watch's sliding look is, and how often it is redone.
#: How long the look waits for the front camera's first frame before giving up (P2Dingo's node takes ~5 s to start).
FRONT_FIRST_FRAME_TIMEOUT_S = 20.0
WATCH_WINDOW_S = 1.5
WATCH_PERIOD_S = 0.5
#: Lidar returns kept: in front of the robot, near enough to matter, base frame.
CROP_BASE = ((-0.2, 2.5), (-1.5, 1.5), (-0.6, 1.2))


def log(msg: str) -> None:
    print(f"[combiner_door {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# --- the arm ----------------------------------------------------------------------------------------------------------

class ArmLink:
    """`arm_link.py`'s socket: targets out, feedback in."""

    def __init__(self, port: int = ARM_LINK_PORT):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=3.0)
        self.sock.settimeout(None)
        self.lock = threading.Lock()
        self.q = None
        self.gripper = None
        self.rx = 0.0
        self.hello = None
        self.errors: list[str] = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        buffer = b""
        while True:
            try:
                chunk = self.sock.recv(65536)
            except OSError:
                return
            if not chunk:
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                with self.lock:
                    if msg.get("type") == "feedback" and msg.get("q") is not None:
                        self.q = np.asarray(msg["q"], dtype=float)
                        self.gripper = msg.get("gripper")
                        self.rx = time.monotonic() - float(msg.get("age_s", 0.0))
                    elif msg.get("type") == "hello":
                        self.hello = msg
                    elif msg.get("type") == "error":
                        self.errors.append(str(msg.get("detail")))
                        log(f"arm link: {msg.get('detail')}")

    def _send(self, msg: dict) -> None:
        self.sock.sendall((json.dumps(msg) + "\n").encode())

    def latest(self):
        with self.lock:
            return (None if self.q is None else self.q.copy()), self.gripper, \
                (time.monotonic() - self.rx if self.rx else math.inf)

    def target(self, q_servo_deg, gripper_units, speed_deg_s: float = plan.ARM_SPEED_DEG_S) -> None:
        self._send({"type": "target", "q": [round(float(v), 3) for v in q_servo_deg],
                    "gripper": None if gripper_units is None else round(float(gripper_units), 2),
                    "speed_deg_s": float(speed_deg_s)})

    def keepalive(self) -> None:
        self._send({"type": "keepalive"})

    def hold(self) -> None:
        try:
            self._send({"type": "hold"})
        except OSError:
            pass


class VirtualArm:
    """A dry run's arm: folded at first, then moving towards where it was last sent as the arm link moves the real one
    (`arm_link.SimulatedArm`: the link's profile and a model of the servo)."""

    def __init__(self):
        self.sim = arm_link.SimulatedArm(d1_hardware.REST_SERVO_DEG)

    @property
    def q(self):
        return self.sim.q

    @q.setter
    def q(self, value):
        self.sim = arm_link.SimulatedArm(value, self.sim.gripper)

    def latest(self):
        return self.sim.q.copy(), self.sim.gripper, 0.0

    def target(self, q_servo_deg, gripper_units, speed_deg_s: float = plan.ARM_SPEED_DEG_S) -> None:
        self.sim.step(q_servo_deg, gripper_units, speed_deg_s)

    def keepalive(self) -> None:
        pass

    def hold(self) -> None:
        pass


# --- the node ---------------------------------------------------------------------------------------------------------

class CombinerDoorNode:
    def __init__(self, args):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
        from std_msgs.msg import Bool, String
        from tf2_msgs.msg import TFMessage
        from visualization_msgs.msg import Marker, MarkerArray

        self.msg = dict(TransformStamped=TransformStamped, CompressedImage=CompressedImage, PointCloud2=PointCloud2,
                        PointField=PointField, String=String, TFMessage=TFMessage, Marker=Marker,
                        MarkerArray=MarkerArray)
        self.args = args
        self.rclpy = rclpy
        self.fix = None
        self.attempt = None
        # No rclpy signal handling: it shuts the context down on Ctrl-C before `spin`'s way out can send StopMove
        # ("publisher's context is invalid", 2026-10-20). Python's own KeyboardInterrupt reaches `spin` instead.
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init()
        self.node = Node("combiner_door")
        node = self.node
        self.lock = threading.RLock()
        self.t0 = time.monotonic()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)

        # State the callbacks fill.
        self.K = None                     # the RealSense's intrinsics: the keyhole's; the look's with --camera wrist|head
        self.detector = None              # the look's tag detector
        self.look_K = None                # the look camera's field, as RViz draws it
        self.last_bezel = None            # the insertion's latest keyhole search (`keyhole.Bezel`), drawn on the wrist frames
        self.keyhole_wide_m = args.keyhole_wide_mm / 1000.0    # the image's own search round the model's (0: none)
        self._mask = None                 # the gripper's jaw in the wrist frames (`bezel_image.gripper_mask`)
        self._kh_saved = -math.inf        # when a keyhole search's frame was last kept
        self._kh = {"body0": None, "searches": 0, "found": 0, "last_log": 0.0}   # `--stage keyhole`'s
        self.look_size = None
        self._last_frame_t = {"look": -1.0, "realsense": -1.0}
        self.front_frames = 0             # frames received from the front camera, counted before any throttle
        if args.camera == "front":
            model = front_camera.camera_model()
            self.detector = TagDetector(model.intrinsic_matrix, fisheye_distortion=model.distortion)
            self.look_K, self.look_size = front_camera.frustum_matrix(model), (model.width, model.height)
            log(f"look: the Go2's front camera {model.width}x{model.height} on {front_camera.TOPIC}, lens: "
                + ("calibrated" if model.calibrated else model.source.split(":")[0]))
        self.frames: list = []            # (t, base pose, [TagDetection], camera pose in the base)
        self.scans: list = []             # (t, base pose, base-frame points)
        self.last_scan_t = 0.0
        self.odom = None                  # (t, 4x4)
        self.sport_state = None
        self.legs = np.zeros(12)
        self.web_teleop = False
        self.debug_jpeg = None
        self.door_reading = mid360.NO_READING
        self._door_prior = 0.0
        self._door_busy = False

        # Inputs.
        # Each heavy input in its own callback group, and the odometry in its own: in the node's one default group every
        # callback waits for the one before, and the odometry came 0.1-0.4 s apart through a walk (the insertion's
        # third live run, 2026-10-21) behind the camera's tag detection and the lidar.
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        self.odom_group, image_group, lidar_group, front_group = (MutuallyExclusiveCallbackGroup() for _ in range(4))
        node.create_subscription(CameraInfo, "/realsense/color/camera_info", self._on_info, qos_profile_sensor_data)
        node.create_subscription(Image, "/realsense/color/image_raw", self._on_image, qos_profile_sensor_data,
                                 callback_group=image_group)
        if args.camera == "front":
            node.create_subscription(CompressedImage, front_camera.TOPIC, self._on_front_image,
                                     qos_profile_sensor_data, callback_group=front_group)
        node.create_subscription(PointCloud2, "/livox/lidar", self._on_lidar, qos_profile_sensor_data,
                                 callback_group=lidar_group)
        node.create_subscription(Bool, "/web_teleop_enabled", self._on_teleop, 10)
        try:
            from unitree_go.msg import LowState, SportModeState
            node.create_subscription(SportModeState, "/sportmodestate", self._on_sport, qos_profile_sensor_data,
                                     callback_group=self.odom_group)
            node.create_subscription(LowState, "/lf/lowstate", self._on_lowstate, qos_profile_sensor_data,
                                     callback_group=self.odom_group)
        except ImportError:
            log("no unitree_go messages: sport mode's odometry and the legs are unavailable")
            if args.odom == "sport":
                raise
        self.tf_buffer = None
        if args.odom == "tf":
            import tf2_ros
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        # Outputs.
        self.pub_tf = node.create_publisher(TFMessage, "/tf", 50)
        self.pub_markers = node.create_publisher(MarkerArray, "/combiner_door/markers", 5)
        self.pub_status = node.create_publisher(String, "/combiner_door/status", latched)
        self.pub_desc = node.create_publisher(String, "/combiner_door/robot_description", latched)
        self.pub_face = node.create_publisher(PointCloud2, "/combiner_door/face_points", 5)
        self.pub_image = node.create_publisher(CompressedImage, "/combiner_door/debug_image/compressed", 5)
        self.sport = None
        if args.live:
            from unitree_api.msg import Request
            self.Request = Request
            self.sport = node.create_publisher(Request, "/api/sport/request", 10)

        # The robot model RViz draws, measured and planned: the URDF's chain, its meshes where the PC has them.
        self.model = viz.UrdfModel(ROOT / "description/go2_d1.urdf")
        desc = String()
        desc.data = viz.description_for_pc(ROOT / "description/go2_d1.urdf", args.mesh_root)
        self.pub_desc.publish(desc)

        # The arm: what the attempt commands (a model in a dry run), and the real arm's feedback, which is drawn. The
        # arm link runs in both (`run_on_dog.sh`); in a dry run it only reports.
        self.arm_feedback = None
        self.arm = VirtualArm() if not args.live else None
        self.arm_sent = None
        self.virtual_base = None
        self.virtual_speed = (0.0, 0.0, 0.0)
        self._connect_feedback()

        # The record.
        stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
        task = "" if args.task == "door" else f"{args.task}_"
        then = f"_door_{args.then_door}" if args.then_door else ""
        self.run_dir = Path(args.run_dir) if args.run_dir else LOG_ROOT / f"{stamp}_{task}{args.stage}{then}{'_live' if args.live else '_dry'}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.trace_file = open(self.run_dir / "trace.csv", "w", newline="")
        self.trace = None
        self.run = {"mode": "combiner_door_robot" if args.task == "door" else "combiner_key_robot", "status": "running",
                    "task": args.task, "stage": args.stage, "then_door": args.then_door, "live": bool(args.live),
                    "camera": args.camera,
                    "look_camera_model": front_camera.camera_model().source if args.camera == "front" else None,
                    "argv": sys.argv[1:], "started_utc": stamp, "odom": args.odom, "events": [],
                    "plan": {"arm_step_deg": plan.ARM_STEP_DEG, "walk_floor": plan.WALK_FLOOR,
                             "walk_cap": plan.WALK_CAP, "carry_s": plan.CARRY_S,
                             "gripper_open_units": round(plan.GRIPPER_OPEN_UNITS, 2),
                             "gripper_shut_units": round(plan.GRIPPER_SHUT_UNITS, 2),
                             "lever_mm": [1000 * GEOMETRY.handle_length, 1000 * GEOMETRY.handle_height,
                                          1000 * GEOMETRY.handle_thickness]}}
        self._write_run()

        # Operator input.
        self.keys: queue.Queue = queue.Queue()
        threading.Thread(target=self._read_keys, daemon=True).start()

        # The wrist camera looks from where the arm holds it: live, the arm first goes to the look pose (`_aim`).
        self.arm_joints, self.arm_links = d1_ik.load_urdf()
        self.look_q = plan.look_pose_servo_deg(self.arm_joints, self.arm_links) if args.camera == "wrist" else None
        self.aimed = args.camera != "wrist" or not args.live or args.release
        self._aim_since: float | None = None
        self._aim_started: float | None = None
        self._last_feedback = None
        self.state = "waiting" if (args.release or not self.aimed) else "watch" if args.stage == "watch" else "waiting"
        self.recovering = False
        self.confirmed: set[str] = set()
        self.abort_reason = ""
        self.paused_for = ""
        self.note = ""
        self.look_started = None
        self.walking = False
        self.stop_sent = 0
        self.posture = ""
        self.virtual_drop = 0.0
        self._odom_stale = False
        self._last_note = ""
        self._last_watch = 0.0
        self._last_markers = 0.0

        node.create_timer(0.1, self._tick)
        node.create_timer(0.1, self._publish_tf)
        node.create_timer(0.5, self._publish_slow)

    # --- callbacks ----------------------------------------------------------------------------------------------

    def now(self) -> float:
        return time.monotonic() - self.t0

    def _on_info(self, msg) -> None:
        if self.K is None:
            self.K = np.asarray(msg.k, dtype=float).reshape(3, 3)
            if self.args.camera != "front":
                self.detector = TagDetector(self.K)
                self.look_K, self.look_size = self.K, (msg.width, msg.height)
            log(f"RealSense {msg.width}x{msg.height}, fx {self.K[0, 0]:.1f} fy {self.K[1, 1]:.1f}")

    def _on_teleop(self, msg) -> None:
        self.web_teleop = bool(msg.data)

    def _on_sport(self, msg) -> None:
        self.sport_state = msg
        if self.args.odom == "sport":
            roll, pitch, yaw = (float(v) for v in msg.imu_state.rpy)
            base = plan.base_pose(float(msg.position[0]), float(msg.position[1]), yaw, roll, pitch)
            with self.lock:
                self.odom = (self.now(), base)

    def _on_lowstate(self, msg) -> None:
        self.legs = np.array([msg.motor_state[i].q for i in range(12)], dtype=float)

    def _odom_now(self):
        if self.args.odom == "tf" and self.tf_buffer is not None:
            try:
                t = self.tf_buffer.lookup_transform("odom", "base_link", self.rclpy.time.Time())
            except Exception:
                return self.odom
            tr, q = t.transform.translation, t.transform.rotation
            rot = viz.quat_to_matrix((q.w, q.x, q.y, q.z))
            base = plan.pose(rot, (tr.x, tr.y, plan.BASE_HEIGHT_M))
            stamp = t.header.stamp.sec + 1e-9 * t.header.stamp.nanosec
            age = max(0.0, self.node.get_clock().now().nanoseconds * 1e-9 - stamp)
            self.odom = (self.now() - age, base)
        return self.odom

    def _base_at_input(self):
        """The base's pose for a camera or lidar frame: the latest odometry, the real one in a dry run too (its
        sensors are the real robot's)."""
        odom = self._odom_now()
        return None if odom is None else odom[1]

    def _due(self, which: str, now: float) -> bool:
        """Whether a camera's next frame is wanted: every frame while looking, else 7.5 a second."""
        if self.state != "looking" and now - self._last_frame_t[which] < 1.0 / 7.5:
            return False
        self._last_frame_t[which] = now
        return True

    def _realsense_shown(self) -> bool:
        """Whether RViz's camera image is the RealSense's although the look is the front camera's: through the
        insertion's key phases, where the wrist camera's view of the lock is what there is to watch."""
        attempt = self.attempt
        if self.state == "keyhole":
            return True
        # `lock_look` too: there the front camera's frames and the wrist's took turns on the one image topic, and
        # RViz's camera panel flickered between them (Lukas, 2026-10-23).
        return isinstance(attempt, insert_plan.InsertAttempt) and (str(attempt.phase).startswith("key_")
                                                                   or attempt.phase == "lock_look")

    def _on_front_image(self, msg) -> None:
        """The Go2's front camera (JPEG from P2Dingo's front camera node): the look's frames."""
        now = self.now()
        self.front_frames += 1
        if self.detector is None or not self._due("look", now):
            return
        base = self._base_at_input()
        if base is None:
            return
        import cv2

        bgr = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        self._look_frame(now, base, plan.FRONT_CAMERA_POSE_BASE, bgr[..., ::-1], msg.header,
                         debug=not self._realsense_shown())

    def _on_image(self, msg) -> None:
        """The RealSense: the insertion's keyhole view, and the look's frames with `--camera wrist|head`."""
        now = self.now()
        attempt = self.attempt
        front = self.args.camera == "front"
        if self.state == "keyhole":
            self._keyhole_frame(now, msg)
            return
        keyhole = isinstance(attempt, insert_plan.InsertAttempt) and attempt.wants_keyhole(now) and self.K is not None
        if front and not keyhole and not (self._realsense_shown() and self.pub_image.get_subscription_count() > 0):
            return
        if not front and self.detector is None:
            return
        if not self._due("realsense", now):
            return
        base = self._base_at_input()
        if base is None:
            return
        camera = self._realsense_in_base()
        if camera is None:
            return
        channels = 3 if msg.encoding in ("rgb8", "bgr8") else 4 if msg.encoding in ("rgba8", "bgra8") else 1
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
        rgb = image[..., :3] if msg.encoding.startswith("rgb") else image[..., 2::-1] if channels > 1 else image[..., 0]
        if keyhole:
            # The insertion's look at the lock (`demos/combiner_key/keyhole.py`), from where the camera is now.
            from demos.combiner_key import keyhole as keyhole_finder

            centre, out, u, t = attempt.keyhole_prediction()
            wide = attempt.keyhole_window_m(self.keyhole_wide_m) if self.keyhole_wide_m > 0 else 0.0
            t0 = time.monotonic()
            bezel = keyhole_finder.find_bezel(rgb, self.K, attempt.base_script(base) @ camera, centre, out, u, t,
                                              wide_m=wide, mask=self._wrist_mask(rgb.shape))
            searched_ms = round(1000 * (time.monotonic() - t0))
            attempt.add_keyhole(bezel)
            self.last_bezel = bezel
            self.run["events"].append({"t": round(now, 3), "phase": attempt.phase, "searched_ms": searched_ms,
                                       "keyhole": bezel.as_dict()})
            self._keep_keyhole_frame(now, attempt, base, camera, rgb)
        if front:
            outlines, text = [], ""
            if isinstance(attempt, insert_plan.InsertAttempt):
                # The keyhole search drawn on the wrist's frame: the model's bezel, the search area, the last result.
                outlines, text = insert_viz.keyhole_outlines(attempt.keyhole_prediction(), self.K,
                                                             attempt.base_script(base) @ camera, self.last_bezel,
                                                             wide_m=attempt.keyhole_window_m(self.keyhole_wide_m))
            self._publish_debug(rgb, [], msg.header, outlines, text)
        else:
            self._look_frame(now, base, camera, rgb, msg.header)

    def _keep_keyhole_frame(self, now, attempt, base, camera, rgb) -> None:
        """A keyhole search's frame with what it found drawn, kept in the run's `keyhole/` (at most one a second): what
        the record's numbers were measured on."""
        if now - self._kh_saved < 1.0:
            return
        self._kh_saved = now
        outlines, text = insert_viz.keyhole_outlines(attempt.keyhole_prediction(), self.K,
                                                     attempt.base_script(base) @ camera, self.last_bezel,
                                                     wide_m=attempt.keyhole_window_m(self.keyhole_wide_m))
        folder = self.run_dir / "keyhole"
        folder.mkdir(exist_ok=True)
        (folder / f"{now:07.2f}_{attempt.phase}.jpg").write_bytes(viz.debug_jpeg(rgb, [], text, outlines=outlines))

    def _wrist_mask(self, shape):
        """The gripper's jaw in wrist frames of `shape`, left out of the image's search."""
        from demos.combiner_key import bezel_image

        if self._mask is None or self._mask.shape != tuple(shape[:2]):
            self._mask = bezel_image.gripper_mask(shape)
        return self._mask

    def _keyhole_frame(self, now, msg) -> None:
        """`--stage keyhole`: a wrist frame searched for the bezel from where the arm holds the camera now, against the
        door the look found; drawn for RViz whatever the arm does (the overlay from its latest pose), searched only
        while it is still (`_realsense_in_base`). Nothing is commanded: move the arm with the website's arm panel."""
        if self.K is None or self.fix is None or not self._due("realsense", now):
            return
        base = self._base_at_input()
        drawn = self._camera_drawn()
        if base is None or drawn is None:
            return
        base = base.copy()
        sport = self.sport_state
        if sport is not None:
            body = float(sport.body_height)
            if self._kh["body0"] is None:
                self._kh["body0"] = body
            base[2, 3] -= max(0.0, self._kh["body0"] - body)      # lying down by hand: the base lowered
        channels = 3 if msg.encoding in ("rgb8", "bgr8") else 4 if msg.encoding in ("rgba8", "bgra8") else 1
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
        rgb = image[..., :3] if msg.encoding.startswith("rgb") else image[..., 2::-1] if channels > 1 else image[..., 0]
        to_script = plan.invert(self.fix.script_odom) @ base
        prediction = insert_plan.keyhole_prediction(self.fix)
        from demos.combiner_key import keyhole as keyhole_finder

        still = self._realsense_in_base()
        w2c = np.linalg.inv(to_script @ drawn)
        uv = keyhole_finder._project(self.K, w2c, np.asarray(prediction[0])[None])[0]
        depth = float((w2c[:3, :3] @ np.asarray(prediction[0]) + w2c[:3, 3])[2])
        grow = self.K[0, 0] * self.keyhole_wide_m / max(depth, 0.05)   # the image search's window, px
        in_view = bool(np.isfinite(uv).all() and -grow <= uv[0] < msg.width + grow
                       and -grow <= uv[1] < msg.height + grow)
        if still is not None and in_view:
            centre, out, u, t = prediction
            self.last_bezel = keyhole_finder.find_bezel(rgb, self.K, to_script @ still, centre, out, u, t,
                                                        wide_m=self.keyhole_wide_m, mask=self._wrist_mask(rgb.shape))
            self._kh["searches"] += 1
            self._kh["found"] += int(self.last_bezel.ok)
            if now - self._kh["last_log"] >= 1.0:
                self._kh["last_log"] = now
                self.run["events"].append({"t": round(now, 3), "keyhole": self.last_bezel.as_dict()})
        outlines, text = insert_viz.keyhole_outlines(prediction, self.K, to_script @ drawn,
                                                     self.last_bezel if in_view else None, wide_m=self.keyhole_wide_m)
        state = "the model's bezel is out of the wrist camera's view" if not in_view else (
            "arm moving: not searched" if still is None else f"searched {self._kh['found']}/{self._kh['searches']} found")
        self.note = state
        self._publish_debug(rgb, [], msg.header, outlines, f"{text} | {state}")

    def _look_frame(self, now, base, camera, rgb, header, debug: bool = True) -> None:
        """One frame of the look's camera: the tag searched for, kept with where the base and the camera were."""
        found = self.detector.detect(np.ascontiguousarray(rgb))
        with self.lock:
            self.frames.append((now, base, found, camera))
            self.frames = [f for f in self.frames if now - f[0] < max(plan.LOOK_S, WATCH_WINDOW_S) + 1.0]
        if debug:
            self._publish_debug(rgb, found, header)

    def _publish_debug(self, rgb, found, header, outlines=(), text: str = "") -> None:
        if self.pub_image.get_subscription_count() == 0:
            return
        try:
            status = self._status_line()        # read beside the tick, which now runs alongside
        except (RuntimeError, KeyError, AttributeError):
            status = ""
        self.debug_jpeg = viz.debug_jpeg(rgb, found, f"{text} | {status}" if text else status, outlines=outlines)
        out = self.msg["CompressedImage"]()
        out.header = header
        out.format = "jpeg"
        out.data = self.debug_jpeg
        self.pub_image.publish(out)

    def _on_lidar(self, msg) -> None:
        base = self._base_at_input()
        if base is None:
            return
        points = viz.cloud_xyz(msg)
        points = plan.lidar_to_base(points)
        (x0, x1), (y0, y1), (z0, z1) = CROP_BASE
        keep = ((points[:, 0] > x0) & (points[:, 0] < x1) & (points[:, 1] > y0) & (points[:, 1] < y1)
                & (points[:, 2] > z0) & (points[:, 2] < z1))
        now = self.now()
        with self.lock:
            self.scans.append((now, base, points[keep]))
            self.scans = [s for s in self.scans if now - s[0] < max(plan.LOOK_S, WATCH_WINDOW_S, plan.WALK_SETTLE_S) + 1.0]
        attempt = self.attempt
        if attempt is not None and getattr(attempt, "reads_door", True) \
                and attempt.phase in ("grip", "squeeze", "pull", "hold", "release", "ungrasp") and not self._door_busy:
            self._door_busy = True
            threading.Thread(target=self._read_door, args=(attempt, base, points[keep]), daemon=True).start()

    def _read_door(self, attempt, base, points) -> None:
        try:
            box = attempt.fix.script_odom @ perception.site_pose(attempt.fix.site)
            in_box = plan.transform(plan.invert(box) @ base, points)
            reading = mid360.door_angle(in_box, self._door_prior)
            if reading.ok:
                self._door_prior = reading.angle_deg
            self.door_reading = reading
        finally:
            self._door_busy = False

    def _read_keys(self) -> None:
        for line in sys.stdin:
            self.keys.put(line.strip().lower())
        # The operator's terminal is gone (the SSH session dropped, or no terminal at all): a live run stops and quits
        # rather than carrying on with nobody to press `a`.
        if self.args.live:
            log("the operator's terminal closed: stopping")
            self.keys.put("q")

    # --- the look -----------------------------------------------------------------------------------------------

    def _make_fix(self, look_odom, frames, scans):
        """The look's fix: the door pull's, or the key's (its stance in front of the lock, its range check)."""
        if self.args.task == "key":
            return key_plan.make_key_fix(look_odom, frames, scans)
        if self.args.task == "insert":
            return insert_plan.make_insert_fix(look_odom, frames, scans)
        return plan.make_fix(look_odom, frames, scans)

    def _confirm_before(self):
        return getattr(self.attempt, "confirm_before", plan.CONFIRM_BEFORE)

    def _look_window(self, since: float):
        with self.lock:
            frames = [(base, found, camera) for t, base, found, camera in self.frames if t >= since]
            scans = [(base, pts) for t, base, pts in self.scans if t >= since]
        return frames, scans

    def _watch(self) -> None:
        now = self.now()
        if now - self._last_watch < WATCH_PERIOD_S or self.odom is None:
            return
        self._last_watch = now
        frames, scans = self._look_window(now - WATCH_WINDOW_S)
        fix = self._make_fix(self.odom[1], frames, scans) if frames else None
        if fix is not None:
            self.fix = fix
            ok, text = fix.in_range()
            self.note = ("IN RANGE: " if ok else "OUT OF RANGE: ") + text
        else:
            self.note = ("waiting for the camera" if self.detector is None else
                         f"no tag in view (the front camera needs the box's root >= {plan.FRONT_LOOK_MIN_ROOT_M:.2f} m "
                         "ahead: step the dog back?)" if self.args.camera == "front" else "no tag in view")

    def _aim(self) -> None:
        """Live, with the camera on the wrist: the arm to the look pose (`plan.look_pose_servo_deg`), in joint space,
        until it is there and still; then the look."""
        if self.look_q is None:
            self._finish("no arm pose aims the wrist camera at the door (plan.LOOK_*): nothing moved")
            return
        try:
            arm = self._arm()
        except RuntimeError as exc:
            self._finish(f"{exc}: nothing moved")
            return
        now = self.now()
        if self._aim_started is None:
            self._aim_started = now
            log("aiming the wrist camera: the arm to the look pose " + str(np.round(self.look_q, 1).tolist()))
        # The jaws shut: open, a finger stands across the middle of the wrist camera's view (Lukas's first look, 2026-10-21).
        arm.target(list(self.look_q), plan.GRIPPER_SHUT_UNITS, plan.JOINT_SPEED_DEG_S)
        q, _, age = arm.latest()
        there = q is not None and age < 0.3 and float(np.max(np.abs(np.asarray(q) - self.look_q))) <= 1.5
        self._aim_since = (now if self._aim_since is None else self._aim_since) if there else None
        if self._aim_since is not None and now - self._aim_since >= 0.5:
            self.aimed = True
            self.run["events"].append({"t": round(now, 3), "aimed": [round(float(v), 2) for v in q]})
            log("aimed")
        elif now - self._aim_started > plan.JOINT_TIMEOUT_S:
            self._finish("the arm did not reach the look pose")

    def _camera_in_base(self):
        """Where the look's camera is in the base for a frame now: the front camera's or the head's fixed mount, or the
        wrist's from the arm's latest measured joints (None while those are stale or the arm is moving)."""
        if self.args.camera == "front":
            return plan.FRONT_CAMERA_POSE_BASE
        return self._realsense_in_base()

    def _realsense_in_base(self):
        """Where the RealSense is in the base for a frame now: the head's mount with `--camera head`, else the wrist's
        from the arm's latest measured joints (None while those are stale or the arm is moving)."""
        if self.args.camera == "head":
            return plan.CAMERA_POSE_BASE
        feedback = self.arm_feedback.latest() if self.arm_feedback is not None else (None, None, None)
        q, _, age = feedback
        if q is None or age > 0.3:
            return None
        q = np.asarray(q, dtype=float)
        last, self._last_feedback = self._last_feedback, q
        if last is not None and float(np.max(np.abs(q - last))) > 0.5:
            return None
        return plan.wrist_camera_base(self.arm_joints, q)

    def _camera_drawn(self):
        """Where RViz draws the look camera's field: the front camera's or the head's mount, or the wrist's from the
        arm's latest feedback."""
        if self.args.camera == "front":
            return plan.FRONT_CAMERA_POSE_BASE
        if self.args.camera == "head":
            return plan.CAMERA_POSE_BASE
        q, _, age = self.arm_feedback.latest() if self.arm_feedback is not None else (None, None, None)
        return None if q is None or age > 1.0 else plan.wrist_camera_base(self.arm_joints, q)

    def _start_look(self) -> None:
        self.state = "looking"
        self.look_started = self.now()
        self.look_base = self.odom[1].copy()
        log("looking: stand still")

    def _finish_look(self) -> None:
        frames, scans = self._look_window(self.look_started)
        fix = self._make_fix(self.look_base, frames, scans)
        record = {"frames": len(frames), "with_tag": sum(1 for f in frames if f[1]), "lidar_frames": len(scans)}
        if fix is None and self.args.stage in ("unfold", "load"):
            # The unfold (and the insertion's hand-in) needs no box: the stance is where the dog stands.
            fix = plan.Fix(plan.level(self.look_base), plan.props.BoxSite(plan.NOMINAL_DISTANCE_M, 0.0, 180.0),
                           np.eye(4), mid360.FaceFit(False, why="no tag"), 0, 0, 0.0, 0.0)
            record["found"] = False
            self.run["look"] = record
            self._start_attempt(fix)
            return
        if fix is None:
            self.run["look"] = record | {"found": False}
            self._finish("no tag in the look: nothing moved")
            return
        ok, text = fix.in_range()
        self.fix = fix
        self.note = text
        record |= {"found": True, "in_range": ok, "text": text, "detections": fix.detections,
                   "site": {"distance_m": round(fix.site.distance_m, 4), "bearing_deg": round(fix.site.bearing_deg, 3),
                            "yaw_deg": round(fix.site.yaw_deg, 3)}, "walk_in_m": round(fix.walk_in_m, 3),
                   "lidar_heading": {"used": fix.fit.ok, "points": fix.fit.points, "span_m": round(fix.fit.span, 3),
                                     "rms_mm": round(1000 * fix.fit.rms, 1) if fix.fit.ok else None,
                                     "why_not": fix.fit.why or None},
                   "tag_only_yaw_off_deg": round(fix.tag_only_yaw_deg, 2)}
        self.run["look"] = record
        self._write_run()
        log(text)
        if self.args.stage == "look":
            self._finish("look only: nothing moved")
            return
        if self.args.stage == "keyhole":
            if not fix.on_a_door():
                self._finish("keyhole: the tag the look found is not on the box's door (the charging dock's?): stand "
                             "the dog in front of the box")
                return
            self.state = "keyhole"
            log("keyhole: the door from this look; every wrist frame searched from where the arm is (nothing is "
                "commanded: move the arm with the website's arm panel). RViz's Camera panel shows it. Ctrl-C to stop.")
            return
        if not ok and self.args.stage not in ("unfold", "load") and not self.args.ignore_range:
            self._finish("the box is out of range: nothing moved")
            return
        self._start_attempt(fix)

    def _start_release(self) -> None:
        """`--release`: no look; let go of whatever the hand holds, from wherever the arm is (`plan.release`)."""
        if self.odom is None:
            self.note = "waiting for odometry"
            return
        feedback = self.arm_feedback.latest() if self.arm_feedback is not None else (None, None, None)
        if feedback[0] is None:
            self.note = "waiting for the arm's feedback"
            return
        if self.args.live:
            try:
                self._arm()
            except RuntimeError as exc:
                self._finish(f"{exc}: nothing moved")
                return
        else:
            # The model starts where the real arm is, its jaws as they report themselves.
            self.arm.sim = arm_link.SimulatedArm(feedback[0], 40.8 if feedback[1] is None else float(feedback[1]))
        base = plan.level(self.odom[1])
        frame = base @ plan.pose(None, (0.0, 0.0, -plan.BASE_HEIGHT_M - plan.props.POST_TOP_M))
        fix = plan.Fix(frame, plan.props.BoxSite(plan.NOMINAL_DISTANCE_M, 0.0, 180.0), np.eye(4),
                       mid360.FaceFit(False, why="no look"), 0, 0, 0.0)
        self.attempt = plan.release(fix, self.odom[1], feedback[0])
        if not self.args.live:
            self.virtual_base = self.odom[1].copy()
        self.state = "attempt"
        log("release (" + ("LIVE" if self.args.live else "dry run: nothing will move") + "): "
            + " -> ".join(ph.name for ph in self.attempt.script.phases))
        if self.args.live and not self.args.yes:
            self._pause("start")

    def _start_attempt(self, fix) -> None:
        if self.args.live:
            try:
                self._arm()
            except RuntimeError as exc:
                self._finish(f"{exc}: nothing moved")
                return
        if self.args.task == "insert":
            self.attempt = insert_plan.InsertAttempt(
                fix, self.args.stage, dry=not self.args.live,
                calibration_path=str(ROOT / insert_plan.CALIBRATION_PATH) if self.args.live else None)
        elif self.args.task == "key":
            self.attempt = key_plan.KeyAttempt(fix, self.args.stage, fold=not self.args.no_fold,
                                               speed_scale=self.args.speed_scale)
        else:
            self.attempt = plan.Attempt(fix, self.args.stage,
                                        plan.scaled_timing(self.args.speed_scale, self.args.door_deg),
                                        fold=not self.args.no_fold)
        if not self.args.live:
            self.virtual_base = self.odom[1].copy()
        self.state = "attempt"
        phases = " -> ".join(p.name for p in self.attempt.script.phases)
        log(f"attempt to {self.args.stage} ({'LIVE' if self.args.live else 'dry run: nothing will move'}): {phases}")
        if self.args.live and not self.args.yes and "start" not in self.confirmed:
            # One Enter for the whole attempt (`--step`: one before each step that moves something).
            self._pause("start")
        else:
            self._classic_walk()

    # --- the attempt --------------------------------------------------------------------------------------------

    def _tick(self) -> None:
        try:
            self._tick_inner()
        except Exception as exc:  # noqa: BLE001 -- a crash mid-attempt must still stop the robot
            log(f"error: {exc!r}")
            self._stop_motion(f"error: {exc!r}")
            self.state = "aborted"
            raise

    def _tick_inner(self) -> None:
        self._handle_keys()
        if self.state == "watch":
            self._watch()
            return
        if self.state == "waiting":
            if self.args.release:
                self._start_release()
                return
            if self.odom is None or self.detector is None:
                self.note = "waiting for odometry and the camera"
                return
            if self.args.camera == "front" and self.front_frames == 0:
                # P2Dingo's node decodes only while subscribed: its first frame comes ~5 s after this node asks (the
                # stream joined, a keyframe awaited). A look started before it saw nothing (2026-10-22, live).
                if self.now() > FRONT_FIRST_FRAME_TIMEOUT_S:
                    self._finish(f"no frame from the front camera in {FRONT_FIRST_FRAME_TIMEOUT_S:.0f} s: is "
                                 "p2dingo-front-camera running (cd ~/P2Dingo && docker compose up -d front_camera)?")
                    return
                self.note = "waiting for the front camera's first frame"
                return
            if not self.aimed:
                if not self.args.yes and "start" not in self.confirmed:
                    self._pause("start")
                    return
                self._aim()
                return
            if self.args.stage == "watch":
                self.state = "watch"
                return
            self._start_look()
            return
        if self.state == "looking":
            if self.now() - self.look_started >= plan.LOOK_S:
                self._finish_look()
            return
        if self.state in ("paused", "aborted", "finished"):
            self._keep_still()
            return
        if self.state == "attempt":
            self._step_attempt()

    def _step_attempt(self) -> None:
        attempt = self.attempt
        script = attempt.script
        # Wait for the operator before a phase that moves something.
        upcoming = attempt.phase if script.elapsed_s == 0.0 and script.index == 0 else (
            attempt.next_phase() if script.phases[script.index].duration_s - script.elapsed_s <= 0.1 + 1e-9 else "")
        if (self.args.live and self.args.step and not self.args.yes and not self.recovering
                and upcoming in self._confirm_before() and upcoming not in self.confirmed):
            self._pause(upcoming)
            return
        must = getattr(attempt, "must_confirm", {})
        if self.args.live and upcoming in must and (id(script), upcoming) not in self.confirmed:
            # Someone has to do something first (hand the key in, hold it): always asked, --yes or not.
            self._pause(upcoming, must[upcoming], key=(id(script), upcoming))
            return
        obs = self._observation()
        if obs.odom_age_s > 0.5 and not self._odom_stale:
            log(f"odometry {obs.odom_age_s:.2f} s old in {attempt.phase}")
            self.run["events"].append({"t": round(obs.now_s, 3), "odom_stale_s": round(obs.odom_age_s, 2),
                                       "phase": attempt.phase})
        self._odom_stale = obs.odom_age_s > 0.5
        if self.args.live and attempt.next_phase() == "walk_in" and not self._standing():
            self._abort("the dog is not standing: stand it up first")
            return
        command = attempt.step(obs)
        self._trace(obs, command)
        self._posture(command.posture)
        if command.note and command.note != self._last_note:
            log(command.note)                    # the insertion's hand-in and hand-back countdowns
        self._last_note = command.note
        if command.finished and self._then_door(attempt):
            return
        if command.finished:
            self._finish("attempt finished")
            return
        if command.abort:
            self._abort(command.abort)
            return
        if (self.args.live and self.args.step and not self.args.yes and not self.recovering
                and command.phase in self._confirm_before() and command.phase not in self.confirmed):
            # A phase begun early, its last one over as soon as the arm got there: wait before sending its first step.
            self._pause(command.phase)
            return
        if self.args.live and command.phase in must and (id(attempt.script), command.phase) not in self.confirmed:
            self._pause(command.phase, must[command.phase], key=(id(attempt.script), command.phase))
            return
        if command.arm_servo_deg is not None:
            arm = self._arm()
            if isinstance(arm, VirtualArm) and isinstance(attempt, insert_plan.InsertAttempt):
                arm.sim.stall_units = insert_plan.BAR_UNITS        # a dry run's key between the jaws
            arm.target(command.arm_servo_deg, command.gripper_units, command.speed_deg_s)
            attempt.sent(command)
            self.arm_sent = command
        self._drive(command.base if command.walking else None)

    def _then_door(self, attempt) -> bool:
        """`--then_door`: the key's attempt over, the door pull from where it left the dog (`key_plan.DoorAfterKey`) if
        the robot's own check says the key turned. Whether that took over the run (or ended it)."""
        if not self.args.then_door or not isinstance(attempt, key_plan.KeyAttempt):
            return False
        check = attempt.turn_check()
        self.run["key_attempt_events"] = attempt.events
        self.run["key"] = check
        if not check.get("verified"):
            self._finish("the key's turn was not verified (the fingers have it at "
                         f"{check.get('measured_deg')} deg): no door pull")
            return True
        self.attempt = key_plan.DoorAfterKey(key_plan.door_fix(attempt.fix), self.args.then_door,
                                             plan.scaled_timing(self.args.speed_scale, self.args.door_deg),
                                             fold=not self.args.no_fold)
        self.confirmed = set()
        ok, text = self.attempt.fix.in_range(check_tag=False)
        self.run["door_after_key"] = {"stage": self.args.then_door, "stance": text}
        self._write_run()
        log(f"the key turned; on to the door pull to {self.args.then_door} from here: "
            + " -> ".join(p.name for p in self.attempt.script.phases))
        return True

    def _observation(self) -> plan.Observation:
        now = self.now()
        odom = self._odom_now()
        q, gripper, arm_age = self._arm().latest()
        base = self.virtual_base if self.virtual_base is not None else odom[1]
        odom_age = 0.0 if self.virtual_base is not None else now - odom[0]
        with self.lock:
            scan = None
            if self.scans and self.scans[-1][0] > self.last_scan_t:
                self.last_scan_t = self.scans[-1][0]
                scan = (self.scans[-1][1], self.scans[-1][2])
        # A dry run's door is the real one, shut while the plan opens it: it is drawn, not checked.
        door = self.door_reading if self.args.live else mid360.NO_READING
        return plan.Observation(now, base, odom_age, q, arm_age, scan, door, self.web_teleop,
                                None if gripper is None else float(gripper), self._body_height())

    def _body_height(self) -> float | None:
        """Sport mode's body height; a dry run's lowered as StandDown would (`insert_plan.LIE_DROP_EST_M`, over 2 s)."""
        real = float(self.sport_state.body_height) if self.sport_state is not None else None
        if self.args.live or real is None:
            return real
        goal = -insert_plan.LIE_DROP_EST_M if self.posture == "lie" else 0.0
        self.virtual_drop += max(-0.01, min(0.01, goal - self.virtual_drop))
        return real + self.virtual_drop

    def _posture(self, posture: str) -> None:
        """Sport mode's posture request (`plan.Command.posture`), once each time it changes; live only."""
        if not posture or posture == self.posture:
            return
        self.posture = posture
        self.run["events"].append({"t": round(self.now(), 3), "posture": posture})
        log(f"posture: {posture} ({POSTURE_API[posture]})")
        if self.sport is not None:
            self._drive(None)
            self._sport(POSTURE_API[posture])

    def _connect_feedback(self) -> None:
        try:
            self.arm_feedback = ArmLink(self.args.arm_port)
        except OSError:
            log(f"no arm link on port {self.args.arm_port}: the arm is not drawn" +
                ("" if not self.args.live else ", and a live attempt will not start"))

    def _arm(self):
        """The arm the attempt commands: the model in a dry run, the arm link live (with feedback, or an error)."""
        if self.arm is None:
            link = self.arm_feedback
            if link is None:
                raise RuntimeError(f"no arm link on port {self.args.arm_port}")
            deadline = time.monotonic() + 3.0
            while link.latest()[0] is None and time.monotonic() < deadline:
                time.sleep(0.05)
            q, _, age = link.latest()
            if q is None or (link.hello or {}).get("observe"):
                raise RuntimeError("the arm link has no feedback from the arm" if q is None else
                                   "the arm link was started to observe only (a dry run's)")
            log(f"arm at {np.round(q, 1).tolist()} ({age:.2f} s old)")
            self.arm = link
        return self.arm

    def _standing(self) -> bool:
        s = self.sport_state
        return s is not None and float(s.body_height) > 0.24

    def _move_virtual_base(self, v) -> None:
        """A dry run's base: it walks the command, and stopped it carries on as the real gait does (`plan.CARRY_S`)."""
        if self.virtual_base is None:
            return
        if v is None:
            decay = max(0.0, 1.0 - 0.1 / plan.CARRY_S)
            v = self.virtual_speed = tuple(c * decay for c in self.virtual_speed)
        else:
            self.virtual_speed = tuple(v)
        x, y, yaw = plan.planar(self.virtual_base)
        c, s = math.cos(yaw), math.sin(yaw)
        self.virtual_base = plan.base_pose(x + 0.1 * (c * v[0] - s * v[1]), y + 0.1 * (s * v[0] + c * v[1]),
                                           yaw + 0.1 * v[2])

    def _drive(self, v) -> None:
        """Walk at `v` (vx, vy, wz), or stand (None)."""
        self._move_virtual_base(v)
        if v is not None:
            if self.sport is not None:
                self._sport("move", {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])})
            self.walking = True
            self.stop_sent = 0
            return
        # Standing between walks: a zero Move, as P2Dingo streams while standing. StopMove only when stopping for good
        # (`_stop_motion`): it may take the dog out of classic walk.
        if self.walking or self.stop_sent < 3:
            if self.sport is not None:
                self._sport("move", {"x": 0.0, "y": 0.0, "z": 0.0})
            self.stop_sent += 1
        self.walking = False

    def _classic_walk(self) -> None:
        """Sport mode into classic walk, once, as the attempt starts: it places the feet more precisely than the default
        gait (P2Dingo's `cmdvel_to_unitree`, which sends it as its Move stream starts). Sent on its own, seconds before
        the walk (the unfold comes first), not in front of a Move."""
        stages = TASK_STAGES[self.args.task]
        if self.sport is not None and stages.index(self.args.stage) >= stages.index("walk"):
            self._sport("classic_walk", {"data": True})
            log("gait: classic walk")

    def _sport(self, name: str, parameter: dict | None = None) -> None:
        req = self.Request()
        req.header.identity.id = int(time.time() * 1000) % 2 ** 62
        req.header.identity.api_id = SPORT_API[name]
        req.parameter = json.dumps(parameter) if parameter is not None else ""
        self.sport.publish(req)

    def _keep_still(self) -> None:
        self._drive(None)
        if self.arm is not None:
            try:
                self.arm.keepalive()
            except OSError:
                pass

    def _stop_motion(self, why: str) -> None:
        try:
            self.stop_sent = 0
            self._drive(None)
            if self.sport is not None:
                self._sport("stop_move")
        except Exception as exc:  # noqa: BLE001 -- the launcher sends its own StopMove on the way out too
            log(f"StopMove failed ({exc!r}); the launcher sends another")
        if self.arm is not None:
            self.arm.hold()
        self.run["events"].append({"t": round(self.now(), 3), "stop": why})

    def _pause(self, before: str, what: str = "", key=None) -> None:
        self.state = "paused"
        self.paused_for = before
        self.paused_key = key if key is not None else before
        self._keep_still()
        if what:
            log(f"next: {before}. First {what}; then Enter (a + Enter to abort)")
        elif before == "start":
            log("Enter to start (it then runs through to the end); a + Enter or Ctrl-C stops it at any time")
        else:
            log(f"next: {before}. Enter to go, a + Enter to abort")

    def _abort(self, why: str) -> None:
        self.abort_reason = why
        self._stop_motion(why)
        self.state = "aborted"
        self.run["abort"] = why
        self._write_run()
        log(f"ABORTED: {why}. The base stands, the arm holds. r + Enter: recover (open and back out if at the lever, "
        "then fold); q + Enter or Ctrl-C: quit with the arm where it is")

    def _finish(self, why: str) -> None:
        self._stop_motion(why)
        self.state = "finished"
        self.run["status"] = "finished"
        self.run["outcome"] = why
        self._write_run()
        log(f"{why}. Done; the arm holds where it is")
        # Exit: the main thread stops motion again and writes the record on its way out (`spin`).
        os.kill(os.getpid(), signal.SIGINT)

    def _handle_keys(self) -> None:
        while not self.keys.empty():
            key = self.keys.get()
            if key == "a":
                if self.state in ("attempt", "paused", "looking", "waiting"):
                    self._abort("the operator stopped it")
            elif key == "q":
                # Raised in a timer callback, KeyboardInterrupt never reaches the executor: interrupt the main thread,
                # which stops motion and writes the record on its way out (`spin`).
                self._stop_motion("the operator quit")
                os.kill(os.getpid(), signal.SIGINT)
                return
            elif key == "r" and self.state == "aborted" and self.attempt is not None:
                self.attempt.recover()
                self.recovering = True
                self.abort_reason = ""
                self.state = "attempt"
                log("recovering: " + " -> ".join(p.name for p in self.attempt.script.phases))
            elif key == "" and self.state == "paused":
                self.confirmed.add(self.paused_for)
                self.confirmed.add(getattr(self, "paused_key", self.paused_for))
                self.state = "attempt" if self.attempt is not None else "waiting"
                log(f"go: {self.paused_for}")
                if self.paused_for == "start":
                    self._classic_walk()

    # --- the record ---------------------------------------------------------------------------------------------

    def _trace(self, obs: plan.Observation, command: plan.Command) -> None:
        attempt = self.attempt
        base_s = plan.planar(attempt.base_script(obs.base_odom))
        sent = command.arm_servo_deg or [math.nan] * 6
        measured = obs.arm_servo_deg if obs.arm_servo_deg is not None else [math.nan] * 6
        row = {"t": round(obs.now_s, 3), "phase": command.phase, "base_x_m": round(base_s[0], 4),
               "base_y_m": round(base_s[1], 4), "base_yaw_deg": round(math.degrees(base_s[2]), 2),
               "odom_age_s": round(obs.odom_age_s, 3), "arm_age_s": round(obs.arm_age_s, 3),
               **{f"arm_cmd{i}_deg": round(float(v), 2) for i, v in enumerate(sent)},
               **{f"arm_q{i}_deg": round(float(v), 2) for i, v in enumerate(measured)},
               "gripper_cmd": None if command.gripper_units is None else round(command.gripper_units, 2),
               "gripper_units": None if obs.gripper_units is None else round(obs.gripper_units, 2),
               "base_cmd_vx": round(command.base[0], 3), "base_cmd_vy": round(command.base[1], 3),
               "base_cmd_wz": round(command.base[2], 3), "walking": int(command.walking),
               "ik_status": command.ik_status, "door_lidar_deg": round(obs.door_reading.angle_deg, 2),
               "door_commanded_deg": round(attempt.commanded_door_deg(), 2), "abort": command.abort}
        if isinstance(attempt, insert_plan.InsertAttempt):
            c = attempt.insert_check()
            off, a, b = c.get("key_mm", (None, None, None))
            row |= {"key_off_mm": off, "key_u_mm": a, "key_t_mm": b, "body_height_m": obs.body_height_m,
                    "posture": command.posture, "contact_off_mm": c.get("contact_off_mm"),
                    "searched_r_mm": c.get("searched_r_mm"), "seated": c.get("seated")}
        if isinstance(attempt, key_plan.KeyAttempt):
            check = attempt.turn_check()
            row |= {"key_measured_deg": check.get("measured_deg"), "key_turn_lag_mm": check.get("lag_mm"),
                    "key_arc_deg": round(attempt._arc_deg, 2)}
        if self.trace is None:
            self.trace = csv.DictWriter(self.trace_file, fieldnames=list(row))
            self.trace.writeheader()
        self.trace.writerow(row)
        self.trace_file.flush()          # kept to the last step whatever ends the run (a quit at an abort lost it)

    def _write_run(self) -> None:
        if self.attempt is not None:
            key = "door_attempt_events" if isinstance(self.attempt, key_plan.DoorAfterKey) else "attempt_events"
            self.run[key] = self.attempt.events
        (self.run_dir / "run.json").write_text(json.dumps(self.run, indent=2, default=str))

    # --- what RViz draws ----------------------------------------------------------------------------------------

    def _status_line(self) -> str:
        phase = self.attempt.phase if self.attempt is not None else self.state
        mode = "LIVE" if self.args.live else "DRY RUN"
        mode += f" | {self.args.stage}" + (f" | {phase}" if phase != self.args.stage else "")
        extra = f" | paused before {self.paused_for}" if self.state == "paused" else ""
        extra += f" | ABORTED: {self.abort_reason}" if self.state == "aborted" else ""
        door = self.door_reading.angle_deg
        door_text = f" | door (lidar) {door:.0f} deg" if math.isfinite(door) else ""
        if isinstance(self.attempt, key_plan.KeyAttempt) or (self.attempt is None and self.args.task == "key"):
            key = key_viz.status_text(self.attempt)
            door_text = f" | {key}" if key else ""
        if isinstance(self.attempt, insert_plan.InsertAttempt) or (self.attempt is None and self.args.task == "insert"):
            key = insert_viz.status_text(self.attempt)
            door_text = f" | {key}" if key else ""
        return f"{mode}{extra}{door_text}"

    def _publish_tf(self) -> None:
        odom = self._odom_now()
        if odom is None:
            return
        stamp = self.node.get_clock().now().to_msg()
        transforms = []
        base = odom[1]
        if self.args.odom == "sport":
            # odom -> livox_frame, as GLIM publishes it: P2Dingo's static livox_frame -> base_link hangs the base off it.
            lidar = base @ plan.pose(None, plan.LIDAR_IN_BASE_M)
            transforms.append(viz.transform_msg(self.msg["TransformStamped"], stamp, "odom", "livox_frame", lidar))
        q_arm, gripper, age = (self.arm_feedback.latest() if self.arm_feedback is not None else (None, None, None))
        if q_arm is not None and age > 1.0:
            q_arm = None              # stale: not drawn rather than drawn wrong
        joints = viz.joint_values(LEG_JOINTS, self.legs, q_arm, gripper)
        transforms.append(viz.transform_msg(self.msg["TransformStamped"], stamp, "base_link", "d1/base_link", np.eye(4)))
        transforms += viz.chain_msgs(self.model, joints, "d1/", self.msg["TransformStamped"], stamp)
        # The plan's ghost: the commanded arm on the planned (dry run) or real base; until something is commanded, the
        # robot as it is (so the display is never in error).
        ghost_base = self.virtual_base if self.virtual_base is not None else base
        if self.arm_sent is not None:
            ghost = viz.joint_values(LEG_JOINTS, self.legs, self.arm_sent.arm_servo_deg, self.arm_sent.gripper_units)
        else:
            ghost = joints
        transforms.append(viz.transform_msg(self.msg["TransformStamped"], stamp, "odom", "d1_plan/base_link",
                                            ghost_base))
        transforms += viz.chain_msgs(self.model, ghost, "d1_plan/", self.msg["TransformStamped"], stamp)
        fix = self.attempt.fix if self.attempt is not None else self.fix
        if fix is not None:
            transforms.append(viz.transform_msg(self.msg["TransformStamped"], stamp, "odom", "combiner_door/script",
                                                fix.script_odom))
        out = self.msg["TFMessage"]()
        out.transforms = transforms
        self.pub_tf.publish(out)

    def _publish_slow(self) -> None:
        stamp = self.node.get_clock().now().to_msg()
        attempt = self.attempt
        fix = attempt.fix if attempt is not None else self.fix
        status = {"state": self.state, "stage": self.args.stage, "live": bool(self.args.live),
                  "phase": attempt.phase if attempt is not None else None, "note": self.note,
                  "paused_before": self.paused_for if self.state == "paused" else None,
                  "abort": self.abort_reason or None, "door_lidar_deg": self.door_reading.angle_deg,
                  "door_commanded_deg": attempt.commanded_door_deg() if attempt is not None else None,
                  "run_dir": str(self.run_dir)}
        if isinstance(attempt, key_plan.KeyAttempt):
            status["key"] = attempt.turn_check()
        if isinstance(attempt, insert_plan.InsertAttempt):
            status["insert"] = attempt.insert_check()
        if fix is not None:
            status["site"] = {"distance_m": round(fix.site.distance_m, 3), "bearing_deg": round(fix.site.bearing_deg, 2),
                              "yaw_deg": round(fix.site.yaw_deg, 2), "walk_in_m": round(fix.walk_in_m, 3),
                              "lidar_heading": fix.fit.ok, "lidar_rms_mm": round(1000 * fix.fit.rms, 1)
                              if fix.fit.ok else None, "face_behind_tag_mm": round(1000 * fix.fit.depth, 1)
                              if fix.fit.ok else None, "face_beyond_edges": round(fix.beyond_edges, 2), "tag_vs_lidar_yaw_deg": round(fix.tag_only_yaw_deg, 2),
                              "tag_tilt_deg": round(fix.tag_tilt_deg, 2), "detections": fix.detections}
        sport = self.sport_state
        if sport is not None:
            status["robot"] = {"mode": int(sport.mode), "body_height_m": round(float(sport.body_height), 3),
                               "rpy_deg": [round(math.degrees(float(v)), 2) for v in sport.imu_state.rpy],
                               "standing": self._standing()}
        msg = self.msg["String"]()
        msg.data = json.dumps(status, default=str)
        self.pub_status.publish(msg)
        if fix is None:
            return
        key = isinstance(attempt, key_plan.KeyAttempt) or (attempt is None and self.args.task == "key")
        inserting = isinstance(attempt, insert_plan.InsertAttempt) or (attempt is None and self.args.task == "insert")
        extra = key_viz.extra_for(attempt) if key else insert_viz.extra_for(attempt) if inserting else None
        markers = viz.scene_markers(self.msg["Marker"], stamp, fix, attempt, self._status_line(), self.note,
                                    self.door_reading, self.look_K, self.look_size,
                                    plan_door=not self.args.live, door_task=not (key or inserting), extra=extra,
                                    camera_pose=self._camera_drawn())
        array = self.msg["MarkerArray"]()
        array.markers = markers
        self.pub_markers.publish(array)
        if self.pub_face.get_subscription_count() > 0:
            with self.lock:
                scans = [(base, pts) for t, base, pts in self.scans[-5:]]
            if scans:
                cloud = np.concatenate([plan.transform(plan.invert(fix.script_odom) @ b, p) for b, p in scans])
                window, line = plan.face_inliers(cloud, fix)
                self.pub_face.publish(viz.cloud_msg(self.msg["PointCloud2"], self.msg["PointField"], stamp,
                                                    "combiner_door/script", cloud[window], line[window]))

    # --- running --------------------------------------------------------------------------------------------------

    def spin(self) -> None:
        from rclpy.executors import MultiThreadedExecutor

        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(self.node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self._stop_motion("quit")
            if self.run.get("status") == "running":
                self.run["status"] = "stopped"
            self._write_run()
            self.trace_file.close()
            log(f"record: {self.run_dir}")
            executor.shutdown()
            self.node.destroy_node()
            self.rclpy.try_shutdown()


def stop_only() -> None:
    """StopMove and a zero Move, three times: what `run_on_dog.sh` sends on its way out, whatever became of the node."""
    import rclpy
    from rclpy.node import Node
    from unitree_api.msg import Request

    rclpy.init()
    node = Node("combiner_door_stop")
    pub = node.create_publisher(Request, "/api/sport/request", 10)
    time.sleep(0.5)
    for _ in range(3):
        for api, parameter in ((SPORT_API["move"], {"x": 0.0, "y": 0.0, "z": 0.0}), (SPORT_API["stop_move"], None)):
            req = Request()
            req.header.identity.id = int(time.time() * 1000) % 2 ** 62
            req.header.identity.api_id = api
            req.parameter = json.dumps(parameter) if parameter else ""
            pub.publish(req)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.try_shutdown()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--task", choices=tuple(TASK_STAGES), default="door",
                   help="door (default): the door pull. key: the key turned by the arm (demos/combiner_key/robot/"
                        "key_plan.py): stand the dog in front of the lock, not the box's middle. insert: the key handed "
                        "to the arm and inserted into the lock lying down (insert_plan.py).")
    p.add_argument("--stage", choices=tuple(dict.fromkeys(sum(TASK_STAGES.values(), ()))), default="watch",
                   help="How far to go: watch (default; no attempt), look, then for the door unfold, walk, reach, grip, "
                        "pull; for the key unfold, walk, reach, touch, turn; for the insertion load, walk, lie, reach, "
                        "touch, insert.")
    p.add_argument("--then_door", choices=key_plan.DOOR_STAGES, default=None,
                   help="With --task key --stage turn: once the robot's own check says the key turned, on into the door "
                        "pull to this stage from where the key left the dog (key_plan.DoorAfterKey): the arm folded, it "
                        "walks over to the door pull's stance (back and sideways, no backing off), refits, unfolds, and "
                        "goes on as the door pull. --speed_scale and --door_deg apply to it.")
    p.add_argument("--camera", choices=("front", "wrist", "head"), default="front",
                   help="The look's camera. front (default, since 2026-10-22): the Go2's own front camera, fixed in the "
                        "trunk, so no arm move to aim it (P2Dingo's p2dingo-front-camera container must run; NOMINAL "
                        "lens model). wrist: the RealSense on the D1's bracket (live, the arm first aims it from the look "
                        "pose). head: the RealSense on P2Dingo's head mount. The insertion's keyhole view is always the "
                        "RealSense's, on the wrist unless --camera head.")
    p.add_argument("--keyhole_wide_mm", type=float, default=45.0,
                   help="The insertion's keyhole: look for the bezel in the wrist image first, this far round the "
                        "model's place (demos/combiner_key/bezel_image.py); 0 searches only +-12 mm round the model's.")
    p.add_argument("--live", action="store_true", help="Move the robot. Without it nothing is commanded (a dry run).")
    p.add_argument("--yes", action="store_true", help="With --live: start without waiting for Enter.")
    p.add_argument("--step", action="store_true",
                   help="With --live: wait for Enter before each step that moves something, not only at the start.")
    p.add_argument("--speed_scale", type=float, default=1.0,
                   help="The door script's phases this many times as long as the simulated demo's (default 1.0: its "
                        "own pace; the first live runs were at 1.5).")
    p.add_argument("--door_deg", type=float, default=60.0,
                   help="How far the pull opens the door, degrees (default 60; the first live pulls were 30, the "
                        "simulated demo's is 90).")
    p.add_argument("--odom", choices=("sport", "tf"), default="sport",
                   help="sport: sport mode's /sportmodestate (default). tf: odom -> base_link, GLIM's (P2Dingo's stack).")
    p.add_argument("--no_fold", action="store_true", help="Leave the arm at home at the end instead of folding it.")
    p.add_argument("--release", action="store_true",
                   help="No look, no attempt: open the jaws where they are, back the hand straight out, home, fold. "
                        "For an arm left holding the lever.")
    p.add_argument("--ignore_range", action="store_true",
                   help="Dry runs only: plan the attempt from whatever the look found, in range or not (a tag on "
                        "something other than the box, to try the pipeline).")
    p.add_argument("--arm_port", type=int, default=ARM_LINK_PORT)
    p.add_argument("--mesh_root", default="file:///root/d1training/",
                   help="Where the PC's RViz finds this repo's meshes (view_on_pc.sh mounts it there).")
    p.add_argument("--run_dir", default=None)
    p.add_argument("--stop_only", action="store_true", help="Send StopMove and exit.")
    args = p.parse_args(argv)
    if args.yes and not args.live:
        p.error("--yes only means something with --live")
    stages = TASK_STAGES[args.task]
    if args.stage not in stages:
        p.error(f"--task {args.task} has the stages {', '.join(stages)}")
    if args.then_door and (args.task != "key" or args.stage != "turn"):
        p.error("--then_door follows the key's turn: --task key --stage turn")
    if args.release and args.task != "door":
        p.error("--release lets go of the door's lever; after a key attempt, `r` in its own run lifts the hand off")
    if args.stage == "keyhole" and (args.task != "insert" or args.live):
        p.error("--stage keyhole is the insertion's keyhole search alone, dry: --task insert, no --live")
    if args.ignore_range and args.live:
        p.error("--ignore_range is for dry runs only")
    if not 1.0 <= args.speed_scale <= 4.0:
        p.error("--speed_scale must be 1-4")
    if not 5.0 <= args.door_deg <= 90.0:
        p.error("--door_deg must be 5-90")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.stop_only:
        stop_only()
        return 0
    if args.live and args.stage in ("watch", "look") and not args.release:
        log(f"--stage {args.stage} moves " + ("only the arm, to aim the wrist camera" if args.camera == "wrist"
                                              else "nothing; --live changes nothing"))
    node = CombinerDoorNode(args)
    log(f"{'LIVE' if args.live else 'dry run'}, stage {args.stage}, record in {node.run_dir}")
    node.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())

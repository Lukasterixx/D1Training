"""ROS 2 output for the Go2+D1 testbed: the front L1 lidar, and the state RViz
needs to draw the robot.

Lifted from P2Dingo's `Isaac/go2_omniverse/ros2.py` and trimmed to two jobs:

1. **The L1 lidar.** An RTX lidar on the Go2's nose, publishing a `PointCloud2`
   on `/utlidar/cloud` in frame `utlidar_lidar`. Mount and beam layout are
   P2Dingo's `add_utlidar()` verbatim, so a cloud recorded here is the same
   sensor a cloud recorded there is -- which is the only reason to copy numbers
   rather than pick fresh ones.

2. **Robot state.** `/joint_states` for all 20 joints, `/clock`, `/odom`, and the
   `odom -> base_link` and `base_link -> utlidar_lidar` transforms. Feed
   `/joint_states` to `robot_state_publisher` against `description/go2_d1.urdf`
   and RViz draws the dog and the arm in their real pose; see `run_rviz.sh`.

Everything P2Dingo needs for its nav stack and this repo does not -- the Mid-360,
the thermal camera, GLIM's `map` frame, odom-origin resets, the IMU -- is gone.

Importing this module enables Isaac extensions as a side effect (see below), so
it must be imported *after* `AppLauncher` has started Isaac Sim and only when the
ROS 2 side is actually wanted. `sim.py` imports it lazily for that reason.
"""
from __future__ import annotations

import threading

import numpy as np

# The ROS 2 bridge extension is what puts Isaac's bundled Humble `rclpy` (and the
# message packages) on `sys.path`, and `isaacsim.sensors.rtx` is what defines
# LidarRtx. Both imports below fail without this, so it has to run at import
# time rather than in a setup function.
import omni.kit.app

_EXT_MANAGER = omni.kit.app.get_app().get_extension_manager()
_EXT_MANAGER.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
for _ext in ("isaacsim.core.nodes", "isaacsim.sensors.rtx"):
    try:
        _EXT_MANAGER.set_extension_enabled_immediate(_ext, True)
    except Exception as exc:  # noqa: BLE001 -- optional, and named in the message
        print(f"[ros2][WARN] Could not enable {_ext}: {exc}")

import carb
import omni.replicator.core as rep
import omni.syntheticdata
import omni.timeline
import omni.usd
import rclpy
from geometry_msgs.msg import TransformStamped
from isaacsim.sensors.rtx import LidarRtx
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

BASE_FRAME_ID = "base_link"
ODOM_FRAME_ID = "odom"

# --- Front-mounted Unitree 4D L1 lidar ("utlidar") -------------------------
# The real Go2 publishes this cloud on /utlidar/cloud in frame utlidar_lidar.
# The mount is the official Go2 URDF's radar_joint (parent base_link): forward
# and slightly down, rotated upside-down and tilted up ~15 deg.
UTLIDAR_FRAME_ID = "utlidar_lidar"
UTLIDAR_POINT_CLOUD_TOPIC = "/utlidar/cloud"
UTLIDAR_OFFSET_BASE = np.array([0.28945, 0.0, -0.046825], dtype=float)
UTLIDAR_RPY = (0.0, 2.8782, 0.0)  # radians, from URDF radar_joint
_utlidar_quat_xyzw = Rotation.from_euler("xyz", UTLIDAR_RPY).as_quat()
# (w, x, y, z) ordering, used for both the USD prim orientation and the TF.
UTLIDAR_QUAT_WXYZ = (
    float(_utlidar_quat_xyzw[3]),
    float(_utlidar_quat_xyzw[0]),
    float(_utlidar_quat_xyzw[1]),
    float(_utlidar_quat_xyzw[2]),
)
# Real L1 sensor characteristics: 360 x 90 deg FOV, ~21,600 points/s, 0.05-30 m.
# Unitree spec the vertical FOV as hemispherical: 0 deg (sensor equator) up to
# +90 deg (pole). The upside-down mount then aims that hemisphere forward/down.
UTLIDAR_VERTICAL_MIN_DEG = 0.0
UTLIDAR_VERTICAL_MAX_DEG = 90.0
UTLIDAR_NEAR_RANGE_M = 0.05
UTLIDAR_FAR_RANGE_M = 30.0
UTLIDAR_SCAN_RATE_HZ = 10
UTLIDAR_POINTS_PER_SEC = 21600

# Held so the sensors and writers are not garbage collected mid-run; their
# __del__ throws from a dead stage and stutters the Omniverse console.
_lidars_keep_alive: list = []
_lidar_writers_keep_alive: list = []


def _update_isaac_app_once() -> None:
    try:
        omni.kit.app.get_app().update()
    except Exception:  # noqa: BLE001 -- best effort, one frame of settling
        pass


def _attach_rtx_lidar_ros2_writer(lidar_sensor, topic_name: str, frame_id: str):
    """Publish `lidar_sensor`'s full scan as a PointCloud2.

    Isaac's own replicator writer, rather than the ROS2RtxLidarHelper OmniGraph
    node -- same choice P2Dingo made. The cloud comes out xyz-only (12-byte
    points, no intensity field), which is why `description/go2_d1.rviz` colours it
    by height.
    """
    render_product_path = lidar_sensor.get_render_product_path()

    writer = rep.writers.get("RtxLidarROS2PublishPointCloudBuffer")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=10,
        topicName=topic_name,
    )
    writer.attach([render_product_path])

    try:
        # Publish on every rendered frame rather than every nth.
        omni.syntheticdata.SyntheticData.Get().set_node_attributes(
            "PostProcessDispatchIsaacSimulationGate",
            {"inputs:step": 1},
            render_product_path,
        )
    except Exception as exc:  # noqa: BLE001 -- only costs frame rate, not the cloud
        print(f"[UTLIDAR][WARN] Could not set the simulation gate step: {exc}")

    print(f"[UTLIDAR] publishing PointCloud2 on {topic_name} (frame {frame_id})")
    return writer


def add_utlidar(robot_prim_path: str, debug: bool = False,
                point_cloud_topic: str = UTLIDAR_POINT_CLOUD_TOPIC):
    """Fit the L1 to `robot_prim_path`/base and start publishing its cloud.

    `robot_prim_path` is the articulation root, e.g. `/World/envs/env_0/Robot`.
    The Go2's root body prim is named `base` and is coincident with the URDF's
    `base_link`, so the radar_joint transform applies here unchanged.
    """
    settings = carb.settings.get_settings()
    # The Isaac ROS writer path reads the RTX output buffer from the GPU.
    settings.set_bool("/app/sensors/nv/lidar/outputBufferOnGPU", True)
    # Isaac Lab renders during env.step() only when it has a GUI or believes an
    # RTX sensor exists -- `is_rendering = sim.has_gui() or sim.has_rtx_sensors()`
    # in manager_based_rl_env.py, where has_rtx_sensors() is just this setting.
    # Its own Camera sensor sets it; LidarRtx is created outside Isaac Lab, so
    # nothing does it for us. Without it a headless run renders exactly the
    # warm-up frames below and then stops: the cloud arrives once, /clock freezes
    # at 0.1 s, and both failures look like a bad topic rather than a stalled
    # renderer. P2Dingo never hits this because it always runs with a viewport.
    settings.set_bool("/isaaclab/render/rtx_sensors", True)

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        _update_isaac_app_once()

    # Isaac Sim 5.x resolves this to a shipped OmniLidar asset; every attribute
    # that makes it an L1 rather than a generic spinner is overridden below.
    lidar_prim_path = f"{robot_prim_path}/base/utlidar_sensor"
    lidar_sensor = LidarRtx(
        prim_path=lidar_prim_path,
        translation=tuple(UTLIDAR_OFFSET_BASE),
        orientation=UTLIDAR_QUAT_WXYZ,
        config_file_name="Example_Rotary",
        **{
            "omni:sensor:Core:auxOutputType": "FULL",
            "omni:sensor:Core:outputFrameOfReference": "SENSOR",
        },
    )
    lidar_sensor.initialize()
    _lidars_keep_alive.append(lidar_sensor)

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(lidar_sensor.prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"[UTLIDAR] Invalid lidar prim: {lidar_sensor.prim_path}")

    # Density: points/s ~= numberOfEmitters * reportRateBaseHz. Keep
    # Example_Rotary's native 128-emitter layout and dial the report rate so the
    # output lands near the real L1's ~21,600 points/s.
    num_emitters = 128
    overrides = {
        "omni:sensor:Core:reportRateBaseHz": int(round(UTLIDAR_POINTS_PER_SEC / num_emitters)),
        "omni:sensor:Core:scanRateBaseHz": UTLIDAR_SCAN_RATE_HZ,
        "omni:sensor:Core:nearRangeM": UTLIDAR_NEAR_RANGE_M,
        "omni:sensor:Core:farRangeM": UTLIDAR_FAR_RANGE_M,
        "omni:sensor:Core:auxOutputType": "FULL",
        "omni:sensor:Core:outputFrameOfReference": "SENSOR",
        "omni:sensor:Core:numberOfEmitters": num_emitters,
        "omni:sensor:Core:numberOfChannels": num_emitters,
    }
    for attr_name, value in overrides.items():
        attr = prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            attr.Set(value)
        else:
            print(f"[UTLIDAR][OVERRIDE][MISSING] {attr_name}")

    # Spread the 128 emitters across the L1's 90 deg vertical FOV: 4 azimuth
    # groups x 32 channels, lightly jittered so the rings do not perfectly
    # overlap. That approximates the L1's non-repetitive scan.
    elev_attr_name = "omni:sensor:Core:emitterState:s001:elevationDeg"
    elev_attr = prim.GetAttribute(elev_attr_name)
    if elev_attr and elev_attr.IsValid():
        channels_per_group = 32
        base_stack = np.linspace(
            UTLIDAR_VERTICAL_MIN_DEG, UTLIDAR_VERTICAL_MAX_DEG, channels_per_group
        ).astype(np.float32)
        rng = np.random.default_rng(seed=7)
        all_elev: list[float] = []
        for _ in range(4):
            jitter = rng.normal(0.0, 0.45, channels_per_group).astype(np.float32)
            group = np.clip(
                base_stack + jitter, UTLIDAR_VERTICAL_MIN_DEG, UTLIDAR_VERTICAL_MAX_DEG
            )
            all_elev.extend(np.sort(group).tolist())
        elev_attr.Set(all_elev)
    else:
        print(f"[UTLIDAR][ELEV MISSING] {elev_attr_name}")

    if debug:
        # Viewport draw only; the ROS publish path does not depend on it.
        render_product_path = lidar_sensor.get_render_product_path()
        for writer_name in ("RtxLidarDebugDrawPointCloud", "RtxLidarDebugDrawPointCloudBuffer"):
            try:
                rep.writers.get(writer_name).attach([render_product_path])
                print(f"[UTLIDAR][DEBUG] attached {writer_name}")
                break
            except Exception as exc:  # noqa: BLE001 -- name varies across releases
                print(f"[UTLIDAR][DEBUG][FAILED] {writer_name}: {exc}")

    _lidar_writers_keep_alive.append(
        _attach_rtx_lidar_ros2_writer(
            lidar_sensor, topic_name=point_cloud_topic, frame_id=UTLIDAR_FRAME_ID
        )
    )
    return lidar_sensor


def _sim_time_msg() -> Clock:
    """Isaac's timeline clock, as /clock wants it.

    Everything downstream is stamped from this, so `use_sim_time` consumers stay
    consistent with the lidar's own timestamps.
    """
    sim_time = float(omni.timeline.get_timeline_interface().get_current_time())
    msg = Clock()
    msg.clock.sec = int(sim_time)
    msg.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
    return msg


class RobotStateNode(Node):
    """Publishes the one robot's joint angles, odometry and sensor TF.

    Deliberately single-robot: `--num_envs > 1` puts several dogs in the scene,
    and there is no useful way to draw more than one of them on an unnamespaced
    `/joint_states`. Robot 0 is the one that gets published.
    """

    def __init__(self):
        super().__init__("go2_d1_sim_node")
        qos = QoSProfile(depth=10)
        self.clock_pub = self.create_publisher(Clock, "/clock", qos)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos)
        self.broadcaster = TransformBroadcaster(self, qos=qos)

    def publish_joints(self, joint_names, joint_pos, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(joint_names)
        msg.position = joint_pos.cpu().numpy().astype(float).tolist()
        self.joint_pub.publish(msg)

    def publish_odom(self, pos, quat_wxyz, stamp) -> None:
        """odom -> base_link, straight from the sim's root pose.

        No odom origin games: this is a flat testbed with no SLAM in the loop, so
        the sim's world frame *is* odom and the robot spawns at its origin.
        """
        px, py, pz = (float(v) for v in pos)
        qw, qx, qy, qz = (float(v) for v in quat_wxyz)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = ODOM_FRAME_ID
        tf.child_frame_id = BASE_FRAME_ID
        tf.transform.translation.x = px
        tf.transform.translation.y = py
        tf.transform.translation.z = pz
        tf.transform.rotation.w = qw
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        self.broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME_ID
        odom.child_frame_id = BASE_FRAME_ID
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        self.odom_pub.publish(odom)

    def publish_lidar_tf(self, stamp) -> None:
        """base_link -> utlidar_lidar.

        Published here rather than left to the URDF on purpose: the cloud stays
        placeable by anything listening to /tf, with or without
        robot_state_publisher running. `description/go2_d1.urdf` therefore drops
        the Go2's radar link, so this is the frame's only authority.
        """
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = BASE_FRAME_ID
        tf.child_frame_id = UTLIDAR_FRAME_ID
        tf.transform.translation.x = float(UTLIDAR_OFFSET_BASE[0])
        tf.transform.translation.y = float(UTLIDAR_OFFSET_BASE[1])
        tf.transform.translation.z = float(UTLIDAR_OFFSET_BASE[2])
        tf.transform.rotation.w = float(UTLIDAR_QUAT_WXYZ[0])
        tf.transform.rotation.x = float(UTLIDAR_QUAT_WXYZ[1])
        tf.transform.rotation.y = float(UTLIDAR_QUAT_WXYZ[2])
        tf.transform.rotation.z = float(UTLIDAR_QUAT_WXYZ[3])
        self.broadcaster.sendTransform(tf)


class Ros2Bridge:
    """The ROS 2 side of a run: the L1 sensor plus the state publishers."""

    def __init__(self, env, num_envs: int, lidar_debug: bool = False):
        if num_envs > 1:
            print(f"[ros2] {num_envs} robots in the scene; publishing robot 0 only.")

        self.env = env
        self.lidar = add_utlidar(
            robot_prim_path="/World/envs/env_0/Robot", debug=lidar_debug
        )
        # Replicator's synthetic-data graph -- what turns the lidar's render
        # product into a published PointCloud2 -- is only built the first time the
        # scene renders, so nothing appears on the topic until it has.
        #
        # P2Dingo gets those renders from an env.unwrapped.sim.reset() after its
        # sensor setup. Do NOT copy that: the reset re-creates the physics
        # simulation view on the CPU, and the arm's very next command then dies
        # with "found at least two devices, cuda:0 and cpu". These two renders are
        # the part of reset() that the lidar actually needs (Isaac Lab calls them
        # "warm up replicator buffers" and does the same two).
        for _ in range(2):
            env.unwrapped.sim.render()

        rclpy.init()
        self.node = RobotStateNode()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        print("[ros2] publishing /clock /joint_states /odom /tf and "
              f"{UTLIDAR_POINT_CLOUD_TOPIC}")

    def publish(self) -> None:
        """Push one frame of robot state. Call once per sim step."""
        clock_msg = _sim_time_msg()
        self.node.clock_pub.publish(clock_msg)
        stamp = clock_msg.clock

        robot = self.env.unwrapped.scene["robot"]
        # Root pose is world-absolute; back out env 0's origin so a multi-env
        # scene still reports the robot where it spawned rather than offset.
        origin = self.env.unwrapped.scene.env_origins[0]
        self.node.publish_joints(robot.data.joint_names, robot.data.joint_pos[0], stamp)
        self.node.publish_odom(
            robot.data.root_state_w[0, :3] - origin, robot.data.root_state_w[0, 3:7], stamp
        )
        self.node.publish_lidar_tf(stamp)

    def shutdown(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

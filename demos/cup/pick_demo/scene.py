"""The pick scene: the position-only task's robot and arm model, lying down, with a cup and a wrist camera.

It starts from `position_only.env_cfg.make_cfg` so the arm is the one the task measured and trains
against: force drives with the D1's published torques and measured speed ceiling (`d1_servo`), setpoints
held to 10 Hz, the firmware planner fitted in F-045 between setpoint and drive, and joint feedback
sampled at the D1's 9 Hz. What changes, and why:

- **Posture.** The legs' default is Unitree's lie-down target from `unitree_ros2`'s
  `go2_stand_example.cpp` (hip 0 / 0 / -0.2 / 0.2, thigh 1.36, calf -2.65). Zero leg actions then hold it
  at the stock 25/0.5 gains. The real robot's StandDown posture has not been measured against it.
- **Arm action scale.** The task clips arm actions to +/-1 rad from the zero pose, a bound for a policy.
  The pick needs the whole joint range, so the scale is pi: the same clip then admits every angle inside
  the soft limits, which the action term still enforces. The planner and command hold are untouched.
- **Terminations** are off except the time limit: lying is `low_base` and `base_contact` by definition.
- **Scene.** A rigid cup (`cup_asset`), a pinhole camera on Link6 with a RealSense preset's colour
  intrinsics, a fixed overview camera for the video, and the playback scene's distant light.
- **Camera body.** Intel's own D435 case mesh, drawn at the mount so the assumed pose can be checked
  against the real bracket (`camera_body` for the geometry, `camera_asset` for the USD). Visual only
  -- no collider, no mass -- so it cannot change the physics the pick was measured against. It *can*
  occlude the wrist camera, whose rendered eye sits ~11 mm from the modelled one (F-052), inside the
  case. The camera's near clip (`camera_body.NEAR_CLIP_PAST_HOUSING_M`, 20 mm) keeps the case out of its
  images, as simulated cameras usually are kept from seeing their own housings; the eye is not moved.

The camera renders ideal RGB and depth; `camera.realsense_depth` adds range limits and stereo noise on
the way out.
"""
from __future__ import annotations

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass

from flat_env_cfg import FlatSceneCfg
from position_only.env_cfg import ReachSceneCfg
from demos.common.resting import LYING_LEG_POSE, LYING_SPAWN_HEIGHT_M, make_resting_cfg

from . import camera_body
from .camera import CameraModel, WristMount

OVERVIEW_EYE = (0.95, -0.95, 0.70)
OVERVIEW_TARGET = (0.30, 0.0, 0.10)

_FLAT_SCENE = FlatSceneCfg(num_envs=1, env_spacing=3.0)


@configclass
class PickSceneCfg(ReachSceneCfg):
    light = _FLAT_SCENE.light.copy()
    cup: RigidObjectCfg = None
    wrist_cam: CameraCfg = None
    overview_cam: CameraCfg = None
    # Intel's D435 case mesh at the wrist, as one visual prim (`camera_asset`). None when
    # `--no_camera_body` is passed; the scene skips None entries.
    rs_body: AssetBaseCfg = None


def camera_body_cfg(mount: WristMount, usd_path: str,
                    link6_path: str = "{ENV_REGEX_NS}/Robot/D1/Link6") -> AssetBaseCfg:
    """The `AssetBaseCfg` drawing the RealSense at `mount`.

    The asset is authored in the colour optical frame (`camera_asset`), so the prim takes the mount's
    own pose unchanged: its rotation and its origin in Link6. It is a child of the Link6 prim and so
    follows the wrist without being part of the articulation, and it carries no collider or mass.
    """
    return AssetBaseCfg(
        prim_path=f"{link6_path}/rs_body",
        spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=tuple(float(v) for v in mount.pos_link6),
            rot=tuple(float(v) for v in mount.quat_wxyz())),
    )


def wrist_camera_cfg(camera: CameraModel, mount: WristMount,
                     link6_path: str = "{ENV_REGEX_NS}/Robot/D1/Link6") -> CameraCfg:
    """The wrist RealSense: `camera`'s colour intrinsics at `mount`, rendering RGB and ideal depth.

    Shared by every scene that carries the camera (the pick, the combiner box), so they see through
    the same one. The near clip keeps the housing `camera_body_cfg` draws out of the images (F-052).
    """
    return CameraCfg(
        prim_path=f"{link6_path}/wrist_cam",
        update_period=0.0,
        width=camera.width, height=camera.height,
        data_types=["rgb", "distance_to_image_plane"],
        update_latest_camera_pose=True,   # so the run can compare the rendered camera's pose with the model's
        spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
            intrinsic_matrix=camera.intrinsic_matrix.flatten().tolist(), width=camera.width, height=camera.height,
            clipping_range=(camera_body.NEAR_CLIP_PAST_HOUSING_M, 10.0)),
        offset=CameraCfg.OffsetCfg(pos=tuple(mount.pos_link6), rot=mount.quat_wxyz(), convention="ros"),
    )


def yaw_quat_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def make_pick_cfg(robot_usd: str, cup_usd: str, camera: CameraModel, mount: WristMount, cup_xy=(0.42, 0.03),
                  cup_yaw_deg: float = 0.0, seed: int = 42, device: str = "cuda:0", episode_s: float = 120.0,
                  overview_size=(640, 480), show_camera_body: bool = True, camera_usd: str = None):
    cfg = make_resting_cfg(robot_usd, seed=seed, device=device, episode_s=episode_s)
    robot = cfg.scene.robot

    scene = PickSceneCfg(num_envs=1, env_spacing=3.0)
    scene.robot = robot
    scene.base_contact.update_period = cfg.sim.dt
    scene.arm_contact.update_period = cfg.sim.dt
    scene.cup = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cup",
        spawn=sim_utils.UsdFileCfg(usd_path=cup_usd),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(cup_xy[0], cup_xy[1], 0.001), rot=yaw_quat_wxyz(cup_yaw_deg)),
    )
    scene.wrist_cam = wrist_camera_cfg(camera, mount)
    if show_camera_body:
        if camera_usd is None:
            raise ValueError("show_camera_body needs camera_usd: build it with "
                             "pick_demo.camera_asset.build_camera_usd, which needs the Isaac app.")
        scene.rs_body = camera_body_cfg(mount, camera_usd)
    scene.overview_cam = CameraCfg(
        prim_path="{ENV_REGEX_NS}/OverviewCam",
        update_period=0.0,
        width=overview_size[0], height=overview_size[1],
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=16.0, clipping_range=(0.05, 20.0)),
    )
    cfg.scene = scene
    cfg.sim.physics_material = scene.terrain.physics_material
    cfg.viewer.eye, cfg.viewer.lookat = OVERVIEW_EYE, OVERVIEW_TARGET
    return cfg

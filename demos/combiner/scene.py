"""Isaac Lab scene configuration; import only after AppLauncher starts."""
import math

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass

from demos.combiner.geometry import DEFAULT_HANDLE_TORQUE_NM, GEOMETRY
from demos.common.resting import make_resting_cfg
from demos.cup.pick_demo.scene import camera_body_cfg, wrist_camera_cfg
from flat_env_cfg import FlatSceneCfg
from position_only.env_cfg import ReachSceneCfg

OVERVIEW_EYE = (-0.95, -1.15, 0.95)
OVERVIEW_TARGET = (0.25, 0.0, 0.24)
_FLAT_SCENE = FlatSceneCfg(num_envs=1, env_spacing=3.0)


def overview_view(placement):
    """Orbit with the box bearing so its door faces the overview, even behind the dog."""
    bearing = math.atan2(placement.position[1], placement.position[0])

    def rotate(point):
        x, y, z = point
        return (math.cos(bearing) * x - math.sin(bearing) * y,
                math.sin(bearing) * x + math.cos(bearing) * y, z)

    return rotate(OVERVIEW_EYE), rotate(OVERVIEW_TARGET)


@configclass
class CombinerSceneCfg(ReachSceneCfg):
    light = _FLAT_SCENE.light.copy()
    combiner: ArticulationCfg = None
    overview_cam: CameraCfg = None
    # The cup pick's wrist RealSense and its housing (`demos.cup.pick_demo.scene`).
    wrist_cam: CameraCfg = None
    rs_body: AssetBaseCfg = None


def make_combiner_cfg(robot_usd, box_usd, placement, seed=42, device="cuda:0",
                      door_angle_deg=0.0, handle_angle_deg=0.0, capture=False,
                      camera=None, mount=None, camera_usd=None, handle_torque_nm=DEFAULT_HANDLE_TORQUE_NM):
    cfg = make_resting_cfg(robot_usd, seed=seed, device=device)
    # This is a scene setup, not a reaching episode. Resets are owned by the runner.
    cfg.terminations.time_out = None
    scene = CombinerSceneCfg(num_envs=1, env_spacing=3.0)
    scene.robot = cfg.scene.robot
    scene.base_contact.update_period = cfg.sim.dt
    scene.arm_contact.update_period = cfg.sim.dt
    scene.combiner = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/CombinerBox",
        spawn=sim_utils.UsdFileCfg(
            usd_path=box_usd,
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=8,
                solver_velocity_iteration_count=4)),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=placement.position, rot=placement.quaternion,
            joint_pos={"DoorHinge": math.radians(door_angle_deg),
                       "HandleJoint": math.radians(handle_angle_deg)}, joint_vel={".*": 0.0}),
        actuators={"hinge": ImplicitActuatorCfg(
            joint_names_expr=["DoorHinge"], stiffness=0.0, damping=0.08,
            effort_limit_sim=2.0, velocity_limit_sim=2.0),
            # The return spring. Its rest angle is a position target below the stop (`latch.py` sets it).
            "handle": ImplicitActuatorCfg(
                joint_names_expr=["HandleJoint"], stiffness=GEOMETRY.spring_stiffness(handle_torque_nm),
                damping=0.08, effort_limit_sim=GEOMETRY.spring_effort_limit(handle_torque_nm),
                velocity_limit_sim=3.0)},
    )
    if capture:
        scene.overview_cam = CameraCfg(
            prim_path="{ENV_REGEX_NS}/OverviewCam", update_period=0.0,
            width=960, height=720, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.05, 20.0)))
    if camera is not None:
        scene.wrist_cam = wrist_camera_cfg(camera, mount)
        if camera_usd is not None:
            scene.rs_body = camera_body_cfg(mount, camera_usd)
    cfg.scene = scene
    cfg.sim.physics_material = scene.terrain.physics_material
    cfg.viewer.eye, cfg.viewer.lookat = overview_view(placement)
    return cfg

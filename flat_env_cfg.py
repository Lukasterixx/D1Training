"""Flat testing area for a Go2 carrying a welded-on D1 arm.

Two things differ from Rescue's `custom_rl_env.py`, and both exist to isolate
one question: does the arm's mass break the gait?

1. The terrain is an unconditional flat plane. Rescue's maze is a fine place to
   test navigation and a terrible place to test a payload -- if the dog stumbles
   you cannot tell whether the arm or the wall did it. Flat ground removes the
   confound.

2. The robot is one 20-joint articulation (12 legs + 6 arm + 2 jaws), not the 12
   the policy was trained on. So every joint-space term below is scoped to
   LEG_JOINTS. Miss one and the policy silently receives an 8-wider observation
   vector; rsl_rl will either throw a shape error or, worse, not.

Scoping is safe because `SceneEntityCfg` resolves `joint_names` through
`find_joints(..., preserve_order=False)`, which returns indices in ascending
articulation order. Subsetting cannot reorder the legs relative to each other,
so the 12 values arrive in exactly the order the checkpoint expects.
"""
from __future__ import annotations

import math
import torch

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
import isaaclab.sim as sim_utils
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

from typing import Literal

# The 12 joints the walking checkpoint knows about. Everything joint-space in
# this file is scoped to these; the arm's 8 are driven outside the RL loop.
LEG_JOINTS = [".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"]

D1_ARM_JOINTS = [f"Joint{i}" for i in range(1, 7)]
D1_GRIPPER_JOINTS = ["Joint7_1", "Joint7_2"]

# Arm base offset from the Go2's base link. Same value as Rescue's ARM_MOUNT_Z,
# so the reach clamps and IK targets ported from there still mean what they say.
ARM_MOUNT_Z = 0.08

# Drive gains. Neither of these is Rescue's, and both changes are bug fixes.
#
# These are gains, not physical spec: they say how hard the servo chases its
# target, and PhysX still clamps the torque it produces at the joint's published
# effort limit (3.3 Nm on J0/J1, 1.7 Nm on J2-J5). Raising them buys tracking,
# never strength. They are deliberately NOT scaled with `--arm_mass` -- Unitree
# publishes the D1-550's mass and its per-joint torques independently, so a
# heavier arm does not imply stronger motors.
#
# ARM: Rescue uses 800, which behaves as a soft spring rather than a servo. A
# P-only drive droops by torque/stiffness, so ~1 Nm of gravity against 800 leaves
# several degrees of error on every joint, and those compound down the chain into
# ~13 cm of end-effector error -- the shortfall Rescue documents and blames on
# the IK. It is not the IK. Measured, holding the same target: 100 -> 30.3 cm,
# 400 -> 19.6 cm, 800 -> 13.1 cm, 4000 -> 3.3 cm. A real D1 servo closes its own
# position loop and holds the angle; 4000 models that. The effort limit is doing
# the physical work of saying what the arm cannot lift.
#
# GRIPPER: Rescue drives the jaws at 200 N/m against a 15 N force limit and 30 mm
# of travel -- so the drive only reaches its force limit at 75 mm, past the end
# of the stroke. It is a servo that can never use its motor: at a typical 2.5 mm
# error it makes 0.5 N. Rescue gets away with it because it teleports the arm's
# root and zeroes its velocity every step, so the fingers ride in an artificially
# quiet frame. Welding removes that accidental damper, the fingers feel the arm's
# real motion, and 0.5 N loses -- both get pushed into their travel limits and
# pinned, and the gripper stops responding at all. Measured, not theorised: at
# 200 N/m the drive commands 4.5 N and the joint does not move a micron. 4000 N/m
# reaches 15 N at 3.75 mm, inside the stroke, which is what a jaw servo should do.
# `--selftest` checks the jaws still track, so this cannot regress silently.
ARM_BASE_STIFFNESS = 4000.0
ARM_BASE_DAMPING = 400.0
GRIPPER_BASE_STIFFNESS = 4000.0
GRIPPER_BASE_DAMPING = 400.0

ROBOT_START_POS = (0.0, 0.0, 0.42)


# Velocity commands are injected from the keyboard rather than sampled, exactly
# as in Rescue. The env's own command manager is left in place but frozen.
base_command: dict[str, list[float]] = {}


def constant_commands(env: ManagerBasedRLEnvCfg) -> torch.Tensor:
    """Feed the policy whatever the teleop layer last asked for."""
    tensor_lst = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32, device=env.device).repeat(env.num_envs, 1)
    for i in range(env.num_envs):
        tensor_lst[i] = torch.tensor(
            base_command.get(str(i), [0.0, 0.0, 0.0]), dtype=torch.float32, device=env.device
        )
    return tensor_lst


@configclass
class FlatSceneCfg(InteractiveSceneCfg):
    """Flat ground, one robot, nothing to trip over."""

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        debug_vis=False,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
    )

    robot: ArticulationCfg = None  # filled in by make_env_cfg once the weld exists

    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        drift_range=(0.0, 0.0),
        mesh_prim_paths=["/World/ground"],
        max_distance=100.0,
    )

    # Matches the arm's links too, which is why the spawn cfg must keep
    # activate_contact_sensors=True -- that applies the contact report API to
    # every rigid body in the welded asset, arm included.
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=1800.0, color=(0.55, 0.55, 0.55), visible_in_primary_ray=True
        ),
    )
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(intensity=600.0, color=(1.0, 1.0, 1.0), angle=35.0),
    )


@configclass
class ViewerCfg:
    # Rescue hand-rolls a smoothed chase cam because it needs one that tracks
    # heading through a maze. On flat ground the stock asset_root follow is
    # enough, and eye/lookat are read as offsets from the robot.
    origin_type: Literal["world", "env", "asset_root"] = "asset_root"
    asset_name: str | None = "robot"
    env_index: int = 0
    eye: tuple[float, float, float] = (-3.5, 0.0, 2.5)
    lookat: tuple[float, float, float] = (0.0, 0.0, 0.0)
    cam_prim_path: str = "/OmniverseKit_Persp"
    resolution: tuple[int, int] = (1920, 1080)


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=constant_commands)
        # Scoped: the arm's 8 joints must not reach the policy.
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
        )
        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class ActionsCfg:
    # 12 actions, not 20. `last_action` above inherits this width.
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=LEG_JOINTS, scale=0.25, use_default_offset=True
    )


@configclass
class CommandsCfg:
    # Frozen: constant_commands() is the real source. Kept only because the
    # reward terms below reference it by name.
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(0.0, 0.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0), heading=(0, 0)
        ),
    )


@configclass
class RewardsCfg:
    """Inert during playback -- the manager just has to exist. Kept faithful to
    Rescue so this file can seed an actual training config later."""

    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=1.5, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=0.75, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    dof_torques_l2 = RewTerm(
        func=mdp.joint_torques_l2,
        weight=-0.0002,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
    )
    dof_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-2.5e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
    )
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=0.01,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "command_name": "base_velocity",
            "threshold": 0.5,
        },
    )
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=0.0)


@configclass
class TerminationsCfg:
    # Live testbed: never reset out from under the operator. If the arm's weight
    # tips the dog over, that is the result -- watch it, then press R.
    time_out = None
    base_contact = None


@configclass
class EventCfg:
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 0.8),
            "dynamic_friction_range": (0.6, 0.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)},
            "velocity_range": {
                "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
                "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
            },
        },
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (1.0, 1.0), "velocity_range": (0.0, 0.0)},
    )


@configclass
class Go2D1FlatEnvCfg(ManagerBasedRLEnvCfg):
    scene: FlatSceneCfg = FlatSceneCfg(num_envs=1, env_spacing=2.5)
    viewer: ViewerCfg = ViewerCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        self.scene.contact_forces.update_period = self.sim.dt


def make_robot_cfg(robot_usd_path: str, with_arm: bool = True) -> ArticulationCfg:
    """The Go2's stock cfg, pointed at `robot_usd_path` and given arm drives.

    Everything else about UNITREE_GO2_CFG is left alone -- same leg actuator
    model, same rigid/articulation props, same activate_contact_sensors -- so
    that any change in the gait is attributable to the arm and not to a
    re-tuned quadruped.

    With `with_arm=False` the path points at the stock go2.usd instead of the
    weld, and the arm's joint entries have to come out: Isaac Lab raises on a
    joint regex that matches nothing, so leaving them in would break the
    baseline rather than harmlessly no-op.
    """
    cfg = UNITREE_GO2_CFG.copy()
    cfg.prim_path = "{ENV_REGEX_NS}/Robot"
    cfg.spawn.usd_path = robot_usd_path

    joint_pos = {
        ".*L_hip_joint": 0.1,
        ".*R_hip_joint": -0.1,
        "F[L,R]_thigh_joint": 0.8,
        "R[L,R]_thigh_joint": 1.0,
        ".*_calf_joint": -1.5,
    }
    if with_arm:
        # The arm starts folded at its own zero pose -- where the real D1 homes
        # to on return_to_zero.
        joint_pos["Joint[1-6]"] = 0.0
        joint_pos["Joint7_.*"] = 0.0

    cfg.init_state = ArticulationCfg.InitialStateCfg(
        pos=ROBOT_START_POS,
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos=joint_pos,
        joint_vel={".*": 0.0},
    )

    # Keep the Go2's own leg actuators; add the arm's alongside them.
    cfg.actuators = dict(cfg.actuators)
    if with_arm:
        cfg.actuators["d1_arm"] = ImplicitActuatorCfg(
            joint_names_expr=["Joint[1-6]"],
            stiffness=ARM_BASE_STIFFNESS,
            damping=ARM_BASE_DAMPING,
        )
        cfg.actuators["d1_gripper"] = ImplicitActuatorCfg(
            joint_names_expr=["Joint7_.*"],
            stiffness=GRIPPER_BASE_STIFFNESS,
            damping=GRIPPER_BASE_DAMPING,
        )
    return cfg


def make_env_cfg(
    robot_usd_path: str, num_envs: int = 1, with_arm: bool = True
) -> Go2D1FlatEnvCfg:
    cfg = Go2D1FlatEnvCfg()
    cfg.scene.num_envs = num_envs
    cfg.scene.robot = make_robot_cfg(robot_usd_path, with_arm=with_arm)
    return cfg

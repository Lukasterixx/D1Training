"""Initial P0 preparation: free-space stance and position reaching, 18 actions.

This is a training task, separate from the existing locomotion playback config.
Rewards/ranges are starting values, not validated thesis results.
"""
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs import mdp
from isaaclab.managers import (
    EventTermCfg, ObservationGroupCfg, ObservationTermCfg, RewardTermCfg,
    SceneEntityCfg, TerminationTermCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from flat_env_cfg import FlatSceneCfg, EventCfg, make_robot_cfg
from . import mdp as task_mdp

# An explicit common order for observations, actions and saved run manifests.
LEG_NAMES = [f"{leg}_{joint}_joint" for joint in ("hip", "thigh", "calf")
             for leg in ("FL", "FR", "RL", "RR")]
ARM_NAMES = [f"Joint{i}" for i in range(1, 7)]
CONTROLLED_NAMES = LEG_NAMES + ARM_NAMES


def controlled_joints():
    return SceneEntityCfg("robot", joint_names=CONTROLLED_NAMES, preserve_order=True)


@configclass
class ReachSceneCfg(InteractiveSceneCfg):
    terrain = FlatSceneCfg.terrain.copy()
    sky_light = FlatSceneCfg.sky_light.copy()
    robot = None
    # Separate sensors because the welded D1 bodies are a level deeper.
    base_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/base", history_length=3)
    arm_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/D1/Link.*", history_length=3)


@configclass
class CommandsCfg:
    ee_position = task_mdp.WorldPositionCommandCfg()


@configclass
class ActionsCfg:
    legs = task_mdp.LimitedJointPositionActionCfg(
        asset_name="robot", joint_names=LEG_NAMES, preserve_order=True,
        scale=0.25, use_default_offset=True,
    )
    arm = task_mdp.LimitedJointPositionActionCfg(
        asset_name="robot", joint_names=ARM_NAMES, preserve_order=True,
        scale=1.0, use_default_offset=True,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Actor inputs: only what the real robot measures or computes from its encoders.

        No base linear velocity: the Go2 has no direct sensor for it (as in unitree_rl_lab's
        deployed policies). Noise magnitudes are unitree_rl_lab's Go2 values in physical units;
        they apply only when `robustness="unitree"` enables corruption.
        """
        base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel, params={"asset_cfg": controlled_joints()},
                                       noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel, params={"asset_cfg": controlled_joints()},
                                       noise=Unoise(n_min=-1.5, n_max=1.5))
        target_position = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "ee_position"})
        tip_position = ObservationTermCfg(func=task_mdp.tip_position_b)
        previous_action = ObservationTermCfg(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObservationGroupCfg):
        """Privileged critic inputs, noise-free: adds base linear velocity and joint torques."""
        base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel)
        base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel)
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity)
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel, params={"asset_cfg": controlled_joints()})
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel, params={"asset_cfg": controlled_joints()})
        joint_effort = ObservationTermCfg(func=mdp.joint_effort, params={"asset_cfg": controlled_joints()})
        target_position = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "ee_position"})
        tip_position = ObservationTermCfg(func=task_mdp.tip_position_b)
        previous_action = ObservationTermCfg(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy = PolicyCfg()
    critic = CriticCfg()


# Widths the launcher checks: 3+3+18+18+3+3+18 and 3+3+3+18+18+18+3+3+18.
POLICY_OBS_DIM = 66
CRITIC_OBS_DIM = 87


@configclass
class RewardsCfg:
    reach_coarse = RewardTermCfg(func=task_mdp.position_tracking, weight=2.0, params={"std": 0.15})
    reach_fine = RewardTermCfg(func=task_mdp.position_tracking, weight=1.0, params={"std": 0.04})
    alive = RewardTermCfg(func=mdp.is_alive, weight=0.5)
    failure = RewardTermCfg(func=mdp.is_terminated, weight=-2.0)
    upright = RewardTermCfg(func=mdp.flat_orientation_l2, weight=-1.0)
    base_motion = RewardTermCfg(func=task_mdp.base_motion_l2, weight=-0.2)
    action_rate = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.01)
    joint_vel = RewardTermCfg(func=mdp.joint_vel_l2, weight=-0.0001, params={"asset_cfg": controlled_joints()})
    joint_limits = RewardTermCfg(func=mdp.joint_pos_limits, weight=-1.0, params={"asset_cfg": controlled_joints()})


@configclass
class TerminationsCfg:
    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)
    low_base = TerminationTermCfg(func=task_mdp.base_too_low, params={"minimum_height": 0.15})
    bad_orientation = TerminationTermCfg(func=mdp.bad_orientation, params={"limit_angle": 0.8})
    base_contact = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={"threshold": 1.0, "sensor_cfg": SceneEntityCfg("base_contact", body_names="base")},
    )
    outside_workspace = TerminationTermCfg(func=task_mdp.outside_workspace, params={"distance": 0.75})


@configclass
class RobustEventCfg(EventCfg):
    """unitree_rl_lab's Go2 randomisation, for `robustness="unitree"`.

    Pushes use unitree_rl_lab's upstream ±0.5 m/s (MaiRo's course copy raises it to ±1.0 for
    locomotion). Its joint-velocity reset range is omitted: `reset_joints_by_scale` multiplies
    a zero default velocity, so it has no effect.
    """
    physics_material = EventTermCfg(
        func=mdp.randomize_rigid_body_material, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "static_friction_range": (0.3, 1.2),
                "dynamic_friction_range": (0.3, 1.2), "restitution_range": (0.0, 0.15), "num_buckets": 64},
    )
    add_base_mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base"),
                "mass_distribution_params": (-1.0, 3.0), "operation": "add"},
    )
    randomize_base_com = EventTermCfg(
        func=mdp.randomize_rigid_body_com, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base"),
                "com_range": {"x": (0.0, 0.05), "y": (-0.02, 0.02), "z": (-0.02, 0.02)}},
    )
    push_robot = EventTermCfg(
        func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(5.0, 10.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )


@configclass
class PositionOnlyEnvCfg(ManagerBasedRLEnvCfg):
    scene = ReachSceneCfg(num_envs=64, env_spacing=3.0)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    commands = CommandsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    events = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = False
        self.sim.physics_material = self.scene.terrain.physics_material
        self.episode_length_s = 10.0
        self.scene.base_contact.update_period = self.sim.dt
        self.scene.arm_contact.update_period = self.sim.dt
        self.viewer.eye = (2.0, 2.0, 1.5)
        self.viewer.lookat = (0.25, 0.0, 0.5)


def make_cfg(robot_usd_path, num_envs=64, seed=42, device="cuda:0", tip_offset=(0.0, 0.0, 0.0),
             leg_actuator="unitree", robustness="none", self_collisions=False):
    """`leg_actuator`: "unitree" (measured Go2 envelope) or "dc_motor" (Isaac Lab stock).
    `robustness`: "none" (deterministic, for bring-up) or "unitree" (observation noise and
    unitree_rl_lab randomisation). `self_collisions` lets the arm collide with the Go2 body."""
    if robustness not in ("none", "unitree"):
        raise ValueError(f"Unknown robustness profile: {robustness!r}")
    cfg = PositionOnlyEnvCfg()
    cfg.scene.robot = make_robot_cfg(robot_usd_path, with_arm=True, leg_actuator=leg_actuator)
    # unitree_rl_lab's articulation solver settings (stock Go2: 4 position, 0 velocity iterations).
    props = cfg.scene.robot.spawn.articulation_props
    props.solver_position_iteration_count = 8
    props.solver_velocity_iteration_count = 4
    props.enabled_self_collisions = self_collisions
    if robustness == "unitree":
        cfg.events = RobustEventCfg()
        cfg.observations.policy.enable_corruption = True
    cfg.scene.num_envs = num_envs
    cfg.seed = seed
    cfg.sim.device = device
    cfg.commands.ee_position.tip_offset = tuple(tip_offset)
    return cfg

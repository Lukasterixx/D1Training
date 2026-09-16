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
from motor_model import D1_VELOCITY_LIMIT_RAD_S, arm_trajectory as firmware_trajectory, interface_timing
from . import mdp as task_mdp
from .task_space import SPAWN_HEIGHT_M, ZERO_ACTION_BASE_OFFSET_M
from .tool_point import TOOL_BODY, TOOL_OFFSET_M

# An explicit common order for observations, actions and saved run manifests.
LEG_NAMES = [f"{leg}_{joint}_joint" for joint in ("hip", "thigh", "calf")
             for leg in ("FL", "FR", "RL", "RR")]
ARM_NAMES = [f"Joint{i}" for i in range(1, 7)]
CONTROLLED_NAMES = LEG_NAMES + ARM_NAMES


def controlled_joints():
    return SceneEntityCfg("robot", joint_names=CONTROLLED_NAMES, preserve_order=True)


def leg_joints():
    return SceneEntityCfg("robot", joint_names=LEG_NAMES, preserve_order=True)


def arm_joints():
    return SceneEntityCfg("robot", joint_names=ARM_NAMES, preserve_order=True)


def arm_feedback(quantity):
    # period_steps is set by make_cfg's latency profile; 1 means exact simulator state.
    return {"asset_cfg": arm_joints(), "period_steps": 1, "quantity": quantity}


# configclass turns mutable class attributes into default factories, so the ground and light
# exist only on an instance, not on the class.
_FLAT_SCENE = FlatSceneCfg(num_envs=1, env_spacing=3.0)


@configclass
class ReachSceneCfg(InteractiveSceneCfg):
    terrain = _FLAT_SCENE.terrain.copy()
    sky_light = _FLAT_SCENE.sky_light.copy()
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
    arm = task_mdp.HeldJointPositionActionCfg(
        asset_name="robot", joint_names=ARM_NAMES, preserve_order=True,
        scale=1.0, use_default_offset=True, hold_steps=1,
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
        # Legs report at the policy rate. The D1 reports joint angles only, at 10 Hz under the
        # "estimated" latency profile, so its velocities are differenced from those samples.
        leg_joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel, params={"asset_cfg": leg_joints()},
                                           noise=Unoise(n_min=-0.01, n_max=0.01))
        arm_joint_pos = ObservationTermCfg(func=task_mdp.SampledJointFeedback, params=arm_feedback("pos"),
                                           noise=Unoise(n_min=-0.01, n_max=0.01))
        leg_joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel, params={"asset_cfg": leg_joints()},
                                           noise=Unoise(n_min=-1.5, n_max=1.5))
        arm_joint_vel = ObservationTermCfg(func=task_mdp.SampledJointFeedback, params=arm_feedback("vel"),
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


# Widths the launcher checks: 3+3+(12+6)+(12+6)+3+3+18 and 3+3+3+18+18+18+3+3+18.
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
    # Rotation was free while translation was priced, and every G1a failure was rotational (F-041).
    # Same weight as the linear term: price turning the trunk as dearly as moving it.
    base_angular_motion = RewardTermCfg(func=task_mdp.base_angular_motion_l2, weight=-0.2)
    # Prices the squat that F-019 found: without it nothing in the reward opposes crouching toward a
    # box that sits below the resting tool point. Target is the measured settled stance, not a nominal.
    base_height = RewardTermCfg(func=task_mdp.base_height_l2, weight=-50.0,
                                params={"target_height": ZERO_ACTION_BASE_OFFSET_M[2]})
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


def make_cfg(robot_usd_path, num_envs=64, seed=42, device="cuda:0", tip_offset=TOOL_OFFSET_M, tip_body=TOOL_BODY,
             leg_actuator="unitree", robustness="none", self_collisions=True, latency="estimated",
             arm_actuator="d1_servo", spawn_height=SPAWN_HEIGHT_M, target_ranges=None, arm_trajectory="measured"):
    """`leg_actuator`: "unitree" (measured Go2 envelope) or "dc_motor" (Isaac Lab stock).
    `arm_actuator`: "d1_servo" (explicit published torque and URDF speed limits) or "implicit".
    `latency`: "estimated" (leg command delay; D1 commands and feedback at 10 Hz) or "none".
    `robustness`: "none" (deterministic, for bring-up) or "unitree" (observation noise and
    unitree_rl_lab randomisation). `self_collisions` lets the arm collide with the Go2 body: on by
    default, as in unitree_rl_lab, once Week 1 measured no resting contact from the weld.
    `spawn_height`: base height at reset. `target_ranges`: override the command's target box.
    `arm_trajectory`: "measured" (the D1 firmware's fitted motion planner between setpoint and drive,
    F-045) or "none" (setpoints reach the drive at once, as before 2026-09-17)."""
    if robustness not in ("none", "unitree"):
        raise ValueError(f"Unknown robustness profile: {robustness!r}")
    cfg = PositionOnlyEnvCfg()
    timing = interface_timing(latency, 1.0 / (cfg.sim.dt * cfg.decimation), leg_actuator)
    cfg.scene.robot = make_robot_cfg(robot_usd_path, with_arm=True, leg_actuator=leg_actuator,
                                     arm_actuator=arm_actuator,
                                     leg_delay_steps=tuple(timing["leg_delay_physics_steps"]))
    cfg.actions.arm.hold_steps = timing["arm_command_hold_steps"]
    plan = firmware_trajectory(arm_trajectory)
    if plan is not None:
        cfg.actions.arm.trajectory = dict(plan, velocity_limits_rad_s=[D1_VELOCITY_LIMIT_RAD_S[j] for j in ARM_NAMES])
    for term in (cfg.observations.policy.arm_joint_pos, cfg.observations.policy.arm_joint_vel):
        term.params["period_steps"] = timing["arm_feedback_period_steps"]
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
    cfg.commands.ee_position.body_name = tip_body
    cfg.commands.ee_position.tip_offset = tuple(tip_offset)
    if target_ranges is not None:
        cfg.commands.ee_position.ranges = tuple(tuple(axis) for axis in target_ranges)
    # Reset pose: the task stands up rather than taking playback's drop (see SPAWN_HEIGHT_M).
    x, y, _ = cfg.scene.robot.init_state.pos
    cfg.scene.robot.init_state.pos = (x, y, spawn_height)
    return cfg

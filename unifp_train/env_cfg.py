"""Configuration for the Isaac Lab build of UniFP's position/force task.

The robot, its control law and the observation contract come from `unifp_isaaclab`, which is
verified against the training stack (F-087). What is added here is the surrounding environment:
scene, contact sensing, episode length and the observation widths the learning side needs.
"""
from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

from unifp_isaaclab import interface, robot as robot_mod

from . import observations, task_cfg


def shrink_physx(sim_cfg) -> None:
    """PhysX's GPU contact and pair buffers shrunk (ample for a few hundred environments), so an evaluation or a demo
    fits beside training runs on the shared GPU (`--small_physx`). Not for training at 4096."""
    px = sim_cfg.physx
    px.gpu_max_rigid_contact_count, px.gpu_max_rigid_patch_count = 2**18, 2**16
    px.gpu_found_lost_pairs_capacity, px.gpu_found_lost_aggregate_pairs_capacity = 2**18, 2**20
    px.gpu_total_aggregate_pairs_capacity, px.gpu_collision_stack_size = 2**18, 2**25
    px.gpu_heap_capacity, px.gpu_temp_buffer_capacity = 2**25, 2**23


@configclass
class Go2D1PosForceEnvCfg(DirectRLEnvCfg):
    """UniFP's task on the welded Go2 + D1.

    Widths are the trained contract, not a choice: 18 actions, 32 stacked frames of 76 for the
    actor, 3 stacked frames of 153 for the critic. A policy trained here loads into
    `run_unifp_isaaclab.py` and vice versa.
    """

    decimation = interface.DECIMATION
    episode_length_s = task_cfg.EPISODE_LENGTH_S

    action_space = interface.NUM_ACTIONS
    observation_space = interface.NUM_OBS              # 32 x 76
    state_space = observations.NUM_CRITIC_OBS          # 3 x 153

    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=interface.SIM_DT,
        render_interval=interface.DECIMATION,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=robot_mod.GROUND_STATIC_FRICTION,
            dynamic_friction=robot_mod.GROUND_DYNAMIC_FRICTION,
            restitution=robot_mod.GROUND_RESTITUTION,
        ),
    )

    terrain = AssetBaseCfg(prim_path="/World/ground", spawn=robot_mod.ground_cfg())

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9)),
    )

    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096, env_spacing=2.5, replicate_physics=True)

    robot = robot_mod.make_robot_cfg(usd_path="", prim_path="/World/envs/env_.*/Robot")

    #: Contact forces on every body, at the physics rate. The reward terms need the feet, the
    #: thighs and the trunk, and `update_period=0` means every step rather than every render.
    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*", history_length=3, update_period=0.0,
        track_air_time=False,
    )

    #: Filled in by the launcher once the welded USD has been built.
    robot_usd: str = ""
    #: Joints the articulation may carry beyond UniFP's 20, driven by nothing the policy sends (the insertion
    #: demo's compliant key: `demos/insertion/key.py`). Named, so an unexpected joint still raises.
    passive_joints: tuple[str, ...] = ()

    #: The controlled point, from `task_cfg` -- **not** `interface`'s, which records what the
    #: released checkpoints were trained against. See `task_cfg.TOOL_BODY` for why they differ.
    #: Set these back to `interface.TOOL_BODY` / `interface.TOOL_OFFSET_M` to train or evaluate
    #: against the fingertip the released checkpoints used.
    tool_body: str = task_cfg.TOOL_BODY
    tool_offset_m: tuple[float, float, float] = task_cfg.TOOL_OFFSET_M

    #: Command and reward the gripper's roll about its approach axis (F-095, F-099). Off by
    #: default, so an unmodified run still reproduces the task the released checkpoints trained on.
    #: Turning it on adds `task_cfg.EXTENSION_WEIGHTS` to the reward and starts writing
    #: `commands[CMD_EE_ORN_R]`, a channel the observation has always carried and the task has
    #: never written -- so **no observation or action width changes** and every existing checkpoint
    #: still loads. What changes is what the policy is asked for.
    roll_objective: bool = False
    #: Command and reward the hand's whole pose (`pose_goals.py`): orientation in `CMD_EE_ORN_R/P/Y` as a rotation vector, on
    #: goals drawn from joint angles so the arm can reach them. As with the roll, no width changes and any checkpoint loads.
    pose_objective: bool = False
    #: F-099's objective, the control: the position as UniFP draws it, the orientation uniform over SO(3).
    pose_naive: bool = False
    #: Add the task primitives to the goals (`pose_goals.TASK_MIX`): straight translations, turns in place, arcs about a pivot.
    #: Off: joint lines alone, which is how the frozen `unifp_pose_*` sets were drawn.
    pose_primitives: bool = False
    #: Draw and score the pose goals but leave the orientation channels at zero, for a policy that was never told one: the
    #: position-only policies' uncontrolled hand on the same goals, with inputs they have never seen kept out of them.
    pose_blind: bool = False
    #: Push the hand off its tool point, and let the orientation target give way to the moment (`task_cfg.POSE_WRENCH_ARM_M`).
    pose_wrench: bool = False
    #: Reward a still hand while the goal is held (`rewards.pose_still`). 0: off.
    pose_still_weight: float = 0.0
    #: Reward on the arm's closeness to the joint angles the goal was drawn from (`pose_goals.PoseGoalTrajectory.q_current`): the
    #: inverse kinematics as a teacher, dense where the pose terms are not. Training only, and the policy is still told the pose
    #: alone. Possible because the goals come from joint angles: F-099's draw has none. 0: off.
    pose_joint_weight: float = 0.0
    #: How position and orientation are paid: "separate" (`POSE_WEIGHTS`), "joint" or "keypoints" (one term on both).
    pose_reward: str = "separate"
    #: Multiplies `task_cfg.POSE_WEIGHTS`' two orientation terms; 0 leaves them out, for a run that tracks the goal's
    #: position alone on the same goals.
    pose_orn_scale: float = 1.0

    #: Policy steps of position-only training before the force curriculum opens. Upstream gates on
    #: `global_steps > force_start_step * num_steps_per_env`, and this is that product, so the
    #: default is 8,000 iterations at 24 steps each. Set it to 0 to train with forces from the
    #: first step, or to `None`-equivalent large value to keep them off entirely.
    force_start_step: int = task_cfg.FORCE_START_ITERATION * task_cfg.NUM_STEPS_PER_ENV

    #: Draw each environment's leg armature once at start-up, uniform in this range (kg·m², hip and thigh), the knees'
    #: `knee_armature_factor` times as much (a knee geared 1.917 more reflects 1.917^2, F-168). Empty: as configured.
    #: Any task (until 2026-10-26 the leg task's alone): the dog's leg corner is 0-0.002 (F-227).
    leg_armature_range: tuple[float, float] | None = None
    knee_armature_factor: float = 3.67
    #: Draw each environment's leg joint friction once at start-up, uniform in this range (N·m, hip and thigh; PhysX's
    #: static and dynamic friction effort, solved implicitly, so a joint sticks under less), the knees
    #: `knee_friction_factor` times as much (a friction behind the knee's 1.917 gearing, reflected). 2026-10-23: the
    #: real dog stops where it stops at kp 25 (F-191). Empty: as configured (unitree_rl_lab's 0.01).
    leg_friction_range: tuple[float, float] | None = None
    knee_friction_factor: float = 1.917
    #: Accumulate each leg joint's step-to-step target change for `leg_chatter()` (evaluations; `leg_chatter.py`).
    track_leg_chatter: bool = False
    #: The D1 as the dog's arm is (`arm_link.ArmLink`'s keyword arguments): commands at 9-10 Hz, reports at 9 Hz, the
    #: firmware's planner, its stalls. Empty: UniFP's 50 Hz interface, as every policy before 2026-11-06 trained on.
    #: `arm_servo` is the stiff drive that follows it (`arm_link.swap_arm`, on the robot's config at launch).
    arm_link: dict | None = None
    arm_servo: bool = False
    #: Reward on the squared change in each command the arm receives (rad², per second as the others). UniFP's `action_rate_arm`
    #: reads the 50 Hz output, most of which never reaches a 9 Hz arm; this reads what does. Negative to penalise; 0: none.
    arm_step_weight: float = 0.0
    #: Reward on the wrist's squared distance from its default (rad²). Nothing else fixes it: with `roll_objective` off the hand's orientation
    #: is free, so Joint4-6 wander. On the dog that became a target running away at 90 deg/s (F-255). Negative to penalise; 0: none.
    arm_wrist_weight: float = 0.0
    #: Reward on how far the policy's arm targets run past the arm's last report, beyond `task_cfg.ARM_LEAD_FREE_RAD` (rad², summed
    #: over joints). The dog's arm watch reads exactly this and stops at 25 deg held 0.5 s (F-255); nothing else in training prices it,
    #: and a policy free to lead uses its targets as torques. Negative to penalise; 0: none.
    arm_lead_weight: float = 0.0
    #: Reward on the leg targets' distance from the stance past `task_cfg.LEG_TARGET_FREE_RAD`, rad², summed over joints. The dog stops past
    #: 1.2 rad in the first second and 0.9 after (`press_live`), and a policy on legs that answer 0.1-1 of what they are asked learns to ask
    #: more (2026-11-08: 1.54 rad on the dog's frames). Negative to penalise; 0: none.
    leg_target_weight: float = 0.0
    #: Reward on the base's roll or pitch past `task_cfg.BASE_TILT_FREE_RAD`, rad², the larger of the two. Goals are drawn reachable
    #: from a level base, so a lean buys only speed, and the dog stops at 15 deg (2026-11-08: a pose policy leaned 22). 0: none.
    base_tilt_weight: float = 0.0
    #: Leg joints start this far from default at each reset, uniform, rad. The dog handed over 0.16 rad off the trained stance, and what the
    #: policy asks of the arm swings with the stance (J4 -33 or -49 in the same chain). 0: as every earlier run.
    leg_start_noise_rad: float = 0.0
    #: How much of each leg action reaches the legs, drawn per environment in this range. The dog's joints moved 0.165 of what their targets did over
    #: the first whole-body run, where the legs simulated as trained move 0.51 and no joint friction gets below 0.28: something more than friction,
    #: which this stands for without saying what it is. The policy's own action is what it reads back. Empty: all of it.
    leg_answer_range: tuple[float, float] | None = None
    #: Keep the actuators' reset draws (a delayed actuator's per-environment lag) off the global RNG stream, so a frozen
    #: manifest's goal schedule is the same with or without a command delay (`eval` and `manifest`).
    isolate_actuator_rng: bool = False

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
    #: Keep the actuators' reset draws (a delayed actuator's per-environment lag) off the global RNG stream, so a frozen
    #: manifest's goal schedule is the same with or without a command delay (`eval` and `manifest`).
    isolate_actuator_rng: bool = False

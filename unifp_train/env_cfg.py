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

    #: Policy steps of position-only training before the force curriculum opens. Upstream gates on
    #: `global_steps > force_start_step * num_steps_per_env`, and this is that product, so the
    #: default is 8,000 iterations at 24 steps each. Set it to 0 to train with forces from the
    #: first step, or to `None`-equivalent large value to keep them off entirely.
    force_start_step: int = task_cfg.FORCE_START_ITERATION * task_cfg.NUM_STEPS_PER_ENV

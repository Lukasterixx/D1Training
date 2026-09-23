"""The cup demo's folded-leg posture and measured D1 model."""
import math

# Unitree go2_stand_example.cpp target_pos_1, expressed by joint name.
LYING_LEG_POSE = {
    "FR_hip_joint": 0.0, "FL_hip_joint": 0.0, "RR_hip_joint": -0.2, "RL_hip_joint": 0.2,
    ".*_thigh_joint": 1.36, ".*_calf_joint": -2.65,
}
LYING_SPAWN_HEIGHT_M = 0.18


def make_resting_cfg(robot_usd, seed=42, device="cuda:0", episode_s=120.0):
    # Imported after AppLauncher starts, like the scene configs that call this.
    from position_only.env_cfg import make_cfg

    cfg = make_cfg(robot_usd, num_envs=1, seed=seed, device=device, leg_actuator="unitree",
                   robustness="none", self_collisions=True, latency="estimated",
                   arm_actuator="d1_servo", arm_trajectory="measured",
                   spawn_height=LYING_SPAWN_HEIGHT_M)
    cfg.scene.robot.init_state.joint_pos = {**LYING_LEG_POSE, "Joint[1-6]": 0.0, "Joint7_.*": 0.0}
    cfg.actions.arm.scale = math.pi
    for name in ("low_base", "bad_orientation", "base_contact", "outside_workspace"):
        setattr(cfg.terminations, name, None)
    cfg.episode_length_s = episode_s
    cfg.commands.ee_position.debug_vis = False
    return cfg

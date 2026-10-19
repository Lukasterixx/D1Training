"""The demos' postures (the cup pick's folded legs, the combiner's stance) and measured D1 model."""
import math

# Unitree go2_stand_example.cpp target_pos_1, expressed by joint name.
LYING_LEG_POSE = {
    "FR_hip_joint": 0.0, "FL_hip_joint": 0.0, "RR_hip_joint": -0.2, "RL_hip_joint": 0.2,
    ".*_thigh_joint": 1.36, ".*_calf_joint": -2.65,
}
LYING_SPAWN_HEIGHT_M = 0.18
POSTURES = ("lying", "standing")


def make_resting_cfg(robot_usd, seed=42, device="cuda:0", episode_s=120.0, posture="lying"):
    """The arm on a still dog. `posture`: "lying" (the cup pick's folded legs, `LYING_LEG_POSE`) or "standing"
    (the position-only task's stance: the robot config's default leg pose held under zero action, spawned at
    `task_space.SPAWN_HEIGHT_M`, where the base settles 0.274 m up after sliding ~6 cm back)."""
    if posture not in POSTURES:
        raise ValueError(f"Unknown posture: {posture!r}")
    # Imported after AppLauncher starts, like the scene configs that call this.
    from position_only.env_cfg import make_cfg
    from position_only.task_space import SPAWN_HEIGHT_M

    cfg = make_cfg(robot_usd, num_envs=1, seed=seed, device=device, leg_actuator="unitree",
                   robustness="none", self_collisions=True, latency="estimated",
                   arm_actuator="d1_servo", arm_trajectory="measured",
                   spawn_height=LYING_SPAWN_HEIGHT_M if posture == "lying" else SPAWN_HEIGHT_M)
    if posture == "lying":
        cfg.scene.robot.init_state.joint_pos = {**LYING_LEG_POSE, "Joint[1-6]": 0.0, "Joint7_.*": 0.0}
    cfg.actions.arm.scale = math.pi
    for name in ("low_base", "bad_orientation", "base_contact", "outside_workspace"):
        setattr(cfg.terminations, name, None)
    cfg.episode_length_s = episode_s
    cfg.commands.ee_position.debug_vis = False
    return cfg

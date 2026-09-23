"""Record everything the Isaac Lab port needs to reproduce, from the environment that trained it.

The sim-to-sim port (`unifp_isaaclab/`) rebuilds UniFP's observation vector, action mapping and
torque law on a different simulator. Getting that right is not a matter of reading the code
carefully -- a wrong DOF permutation, a missed scale or a sign on the gait phase all produce a
policy that walks badly rather than one that errors. So this writes the interface out of the
running Isaac Gym environment, per step and in full:

  * the constants the port has to match (DOF names and order, default pose, PD gains, torque
    limits, observation scales, action scale, timestep),
  * the raw state each step (base quaternion, body-frame angular velocity, joint positions and
    velocities, previous actions, gait phase, the 15 commands),
  * what UniFP built from that raw state (the 76-wide single observation), and
  * what the policy did with it (the 2432-wide stacked observation and the 18 actions out).

The per-term reward values are exact from **step 1** onward: they sum to the environment's own
`rew_buf` to 1.3e-06. Step 0 is short by whatever the environment accumulated before the loop
started, because a difference of episode sums has nothing to difference against on the first step.
Drop it.

`tests/test_unifp_interface.py` then checks the port's observation builder against the recorded
raw state, and the port's policy against the recorded stacked observation, with no simulator on
either side. That separates "the port describes the interface correctly" from "the two simulators
behave the same", which is the only question the port is actually asking.

Nothing here changes the environment: same overrides as `play_policy.py` (one robot, no noise, no
randomisation), and the policy is run for its actions exactly as `play_policy.py` runs it.

    python unifp_go2d1/dump_interface.py --task=go2d1_pos_force --load_run <run> \
        --checkpoint 48800 --headless --steps 400 --out /tmp/unifp_interface.npz
"""

import argparse
import os
import sys

UNIFP = os.environ.get("UNIFP_ROOT", os.path.expanduser("~/thesis_b_legacy/UniFP"))
sys.path.insert(0, UNIFP)


def main():
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--steps", type=int, default=400)
    pre.add_argument("--out", required=True, help="path to the .npz to write")
    pre.add_argument("--forces", action="store_true",
                     help="wind global_steps past the force curriculum gate, as play_policy.py does")
    known, rest = pre.parse_known_args()
    sys.argv = [sys.argv[0]] + rest

    import isaacgym  # noqa: F401  (must precede torch)
    import numpy as np
    import torch
    import legged_gym.envs  # noqa: F401  (importing registers the tasks)
    from legged_gym.utils import get_args, task_registry

    args = get_args()
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)

    # Identical to play_policy.py: one robot, no noise, no randomisation.
    env_cfg.env.num_envs = 1
    env_cfg.terrain.num_rows, env_cfg.terrain.num_cols = 5, 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    for flag in ("randomize_friction", "push_robots", "randomize_base_mass", "randomize_leg_mass",
                 "randomize_gripper_mass", "randomize_motor", "randomize_base_com"):
        setattr(env_cfg.domain_rand, flag, False)
    env_cfg.env.test = True

    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    obs = env.get_observations()
    train_cfg.runner.resume = True
    runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args,
                                                      train_cfg=train_cfg)
    policy = runner.get_inference_policy(device=env.device)
    env.play = True

    if known.forces:
        env.global_steps = (env_cfg.commands.force_start_step + 1) * train_cfg.runner.num_steps_per_env

    def np_(t):
        return t.detach().cpu().numpy()

    # The second block is what the Isaac Lab *training* port has to reproduce, as opposed to the
    # policy interface: the gait reference the leg rewards are scored against, the contact and
    # stance masks, the privileged observation the critic reads, and every reward term separately.
    # Porting 27 reward terms by reading them is how silent sign and scale errors get in; this
    # makes each one checkable against the environment that trained the policy.
    rec = {k: [] for k in ("base_quat", "base_ang_vel", "dof_pos", "dof_vel", "prev_actions",
                           "gait_indices", "commands", "obs_single", "obs_stacked", "action",
                           "ee_pos", "ee_goal_cart_world", "base_pos", "torques",
                           "ref_dof_pos", "stance_mask", "contact_mask", "privileged_obs",
                           "base_lin_vel", "projected_gravity", "root_states", "rew_buf",
                           "ee_goal_sphere_cmd", "feet_pos", "feet_contact_forces",
                           "thigh_pos", "feet_vel", "feet_air_time", "last_contacts",
                           "base_yaw_quat", "ee_force_measured", "base_force_measured",
                           "ee_force_cmd", "base_force_cmd", "gripper_force_kp", "base_force_kd")}
    rec_rewards = {}

    info = {}
    previous_sums = {}
    with torch.no_grad():
        for _ in range(known.steps):
            # Read the raw state the observation is built from BEFORE stepping, so the recorded
            # state and the recorded observation describe the same instant.
            rec["base_quat"].append(np_(env.base_quat[0]))
            rec["base_ang_vel"].append(np_(env.base_ang_vel[0]))
            rec["dof_pos"].append(np_(env.dof_pos[0]))
            rec["dof_vel"].append(np_(env.dof_vel[0]))
            rec["prev_actions"].append(np_(env.actions[0, :env.num_actions]))
            rec["gait_indices"].append(np_(env.gait_indices[0]))
            rec["commands"].append(np_(env.commands[0, :15]))
            rec["obs_single"].append(np_(env.obs_history[-1][0]))
            rec["obs_stacked"].append(np_(obs["obs"][0]))
            rec["ee_pos"].append(np_(env.ee_pos[0]))
            rec["ee_goal_cart_world"].append(np_(env.curr_ee_goal_cart_world[0]))
            rec["base_pos"].append(np_(env.base_pos[0]))
            rec["ref_dof_pos"].append(np_(env.ref_dof_pos[0]))
            rec["stance_mask"].append(np_(env._get_gait_phase()[0]))
            rec["contact_mask"].append(
                np_((env.contact_forces[:, env.feet_indices, 2] > 5.)[0]).astype("float32"))
            rec["privileged_obs"].append(np_(obs["privileged_obs"][0]))
            rec["base_lin_vel"].append(np_(env.base_lin_vel[0]))
            rec["projected_gravity"].append(np_(env.projected_gravity[0]))
            rec["root_states"].append(np_(env.root_states[0]))
            rec["ee_goal_sphere_cmd"].append(np_(env.curr_ee_goal_sphere[0]))
            rec["feet_pos"].append(np_(env.rigid_state[0, env.feet_indices, :3]))
            rec["feet_contact_forces"].append(np_(env.contact_forces[0, env.feet_indices, :]))
            rec["thigh_pos"].append(np_(env.rigid_state[0, env.thigh_indices, :3]))
            rec["feet_vel"].append(np_(env.rigid_state[0, env.feet_indices, 7:10]))
            # feet_air_time and last_contacts are mutated *inside* _reward_feet_air_time, so the
            # value read here -- after the previous step's reward -- is exactly the input the next
            # step's reward starts from. They do not take the +1 shift the other state does.
            rec["feet_air_time"].append(np_(env.feet_air_time[0]))
            rec["last_contacts"].append(np_(env.last_contacts[0]).astype("float32"))
            rec["base_yaw_quat"].append(np_(env.base_yaw_quat[0]))
            rec["ee_force_measured"].append(np_(env.forces[0, env.gripper_idx, 0:3]))
            rec["base_force_measured"].append(np_(env.forces[0, env.robot_base_idx, 0:3]))
            rec["ee_force_cmd"].append(np_(env.current_Fxyz_gripper_cmd[0]))
            rec["base_force_cmd"].append(np_(env.current_Fxyz_base_cmd[0]))
            rec["gripper_force_kp"].append(np_(env.gripper_force_kps[0]))
            rec["base_force_kd"].append(np_(env.base_force_kds[0]))

            actions = policy(obs, info)
            rec["action"].append(np_(actions[0]))
            obs, rew, _done, _extras = env.step(actions.detach())
            rec["torques"].append(np_(env.torques[0]))
            rec["rew_buf"].append(np_(rew[0]))
            # Each reward term on its own, taken as the *change* in the environment's own
            # `episode_sums` across the step -- i.e. `_reward_<name>() * scale * dt`, exactly as
            # `compute_reward` accumulated it.
            #
            # Not by calling `_reward_<name>()` again: `_reward_feet_air_time` writes to
            # `self.feet_air_time`, so a second call per step would change the run being recorded
            # and the reference would no longer be the trajectory the policy actually flew.
            for name in env.reward_names:
                rec_rewards.setdefault(name, []).append(
                    float(env.episode_sums[name][0]) - previous_sums.get(name, 0.0))
            previous_sums = {name: float(env.episode_sums[name][0]) for name in env.reward_names}

    consts = dict(
        dof_names=np.array(env.dof_names),
        default_dof_pos=np_(env.default_dof_pos[0]),
        p_gains=np_(env.p_gains[0]) if env.p_gains.dim() > 1 else np_(env.p_gains),
        d_gains=np_(env.d_gains[0]) if env.d_gains.dim() > 1 else np_(env.d_gains),
        torque_limits=np_(env.torque_limits),
        motor_strength=np_(env.motor_strength[0]),
        commands_scale=np_(env.commands_scale),
        action_scale=np.float32(env.cfg.control.action_scale),
        decimation=np.int32(env.cfg.control.decimation),
        dt=np.float32(env.dt),
        sim_dt=np.float32(env.cfg.sim.dt),
        cycle_time=np.float32(env.cfg.rewards.cycle_time),
        num_actions=np.int32(env.num_actions),
        num_single_obs=np.int32(env.cfg.env.num_single_obs),
        frame_stack=np.int32(env.cfg.env.frame_stack),
        gripper_stiffness=np.float32(env.cfg.control.gripper_stiffness),
        gripper_damping=np.float32(env.cfg.control.gripper_damping),
        ee_goal_center_offset=np_(env.ee_goal_center_offset[0]),
        checkpoint=np.int32(args.checkpoint),
        load_run=np.array(str(args.load_run)),
        obs_scale_lin_vel=np.float32(env.obs_scales.lin_vel),
        obs_scale_ang_vel=np.float32(env.obs_scales.ang_vel),
        obs_scale_dof_pos=np.float32(env.obs_scales.dof_pos),
        obs_scale_dof_vel=np.float32(env.obs_scales.dof_vel),
    )

    out = {k: np.asarray(v) for k, v in rec.items()}
    out.update({f"reward_{k}": np.asarray(v) for k, v in rec_rewards.items()})
    out["reward_names"] = np.array(sorted(rec_rewards))
    # NOTE: `_prepare_reward_function` has already multiplied these by dt, so a recorded scale is
    # `config scale x 0.02`. The per-term values above carry that same factor; do not apply dt twice.
    out["reward_scales"] = np.array([env.reward_scales.get(n, 0.0) for n in sorted(rec_rewards)],
                                    dtype="float32")
    out.update(consts)
    os.makedirs(os.path.dirname(os.path.abspath(known.out)) or ".", exist_ok=True)
    np.savez(known.out, **out)
    print(f"[dump] wrote {known.out}: {known.steps} steps, "
          f"obs_single {out['obs_single'].shape}, obs_stacked {out['obs_stacked'].shape}", flush=True)


if __name__ == "__main__":
    main()
